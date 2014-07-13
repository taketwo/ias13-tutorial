/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_SEGMENTATION_RANDOM_WALKER_SEGMENTATION_H
#define PCL_SEGMENTATION_RANDOM_WALKER_SEGMENTATION_H

#include <boost/ref.hpp>
#include <boost/bimap.hpp>
#include <boost/mpl/at.hpp>
#include <boost/mpl/map.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>

#include "graph/point_cloud_graph.h"
#include "graph/voxel_grid_graph_builder.h"

namespace pcl
{

  namespace segmentation
  {

    /** Multilabel semi-automatic point cloud segmentation using random walks.
      *
      * This is based on the algorithm described in "Random Walks for Image
      * Segmentation" by Leo Grady. An implementation of this algorithm suitable
      * for generic graphs (not necessarily with 3D points as vertices) is
      * available in \ref pcl::segmentation::randomWalks().
      *
      * This class makes a bridge between input point cloud and the graph-based
      * back-end algorithm. An average user only needs to supply an input point
      * cloud and a set of seeds. The class will take care of converting the
      * input into a weighted graph, running random walker segmentation, and
      * interpreting its output to form point cloud clusters.
      *
      * The default parameters that are used to construct the graph are adequate
      * for table-top scenes captured by a 3D sensor like Kinect.
      *
      * An advanced user may provide a pre-build graph rather than a point
      * cloud if the defaults are not suitable for the data at hand. See the
      * tools available in the \ref graph "pcl_graph" module for construction of
      * graphs from point clouds (pcl::graph::GraphBuilder) and computing edge
      * weights (pcl::graph::EdgeWeightComputer).
      *
      * \author Sergey Alexandrov
      * \ingroup segmentation */
    template <typename PointT>
    class PCL_EXPORTS RandomWalkerSegmentation : public pcl::PCLBase<PointT>
    {

      private:

        using pcl::PCLBase<PointT>::initCompute;
        using pcl::PCLBase<PointT>::deinitCompute;
        using pcl::PCLBase<PointT>::indices_;
        using pcl::PCLBase<PointT>::input_;

        /* The piece of MPL code below is used to relate the possible point
         * types (with which the class template may be instantiated) with
         * corresponding point types augmented with normal+curvature fields. */
        typedef boost::mpl::map
                <
                  boost::mpl::pair<pcl::PointXYZ,          pcl::PointNormal>,
                  boost::mpl::pair<pcl::PointNormal,       pcl::PointNormal>,
                  boost::mpl::pair<pcl::PointXYZRGB,       pcl::PointXYZRGBNormal>,
                  boost::mpl::pair<pcl::PointXYZRGBA,      pcl::PointXYZRGBNormal>,
                  boost::mpl::pair<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>,
                  boost::mpl::pair<pcl::PointXYZI,         pcl::PointXYZINormal>,
                  boost::mpl::pair<pcl::PointXYZINormal,   pcl::PointXYZINormal>
                > PointTypeAssociations;
        BOOST_MPL_ASSERT ((boost::mpl::has_key<PointTypeAssociations, PointT>));

      public:

        /* Public typedefs for point and point cloud types. */
        typedef PointT                                                       Point;
        typedef pcl::PointCloud<Point>                                       PointCloud;
        typedef typename PointCloud::Ptr                                     PointCloudPtr;
        typedef typename PointCloud::ConstPtr                                PointCloudConstPtr;
        typedef typename boost::mpl::at<PointTypeAssociations, PointT>::type PointWithNormal;
        typedef pcl::PointCloud<PointWithNormal>                             PointWithNormalCloud;
        typedef typename PointWithNormalCloud::Ptr                           PointWithNormalCloudPtr;

        typedef pcl::search::Search<PointWithNormal> Search;
        typedef typename Search::Ptr SearchPtr;

        /* Public typedefs related with graph. */
        typedef boost::subgraph
                <pcl::graph::point_cloud_graph
                <PointWithNormal,
                 boost::vecS,
                 boost::undirectedS,
                 boost::property<boost::vertex_color_t, uint32_t>,
                 boost::property<boost::edge_weight_t, float,
                 boost::property<boost::edge_index_t, int> > > >           Graph;
        typedef typename boost::graph_traits<Graph>::vertex_descriptor     VertexId;
        typedef typename boost::graph_traits<Graph>::edge_descriptor       EdgeId;
        typedef typename boost::graph_traits<Graph>::vertex_iterator       VertexIterator;
        typedef typename boost::graph_traits<Graph>::edge_iterator         EdgeIterator;
        typedef typename boost::graph_traits<Graph>::adjacency_iterator    AdjacencyIterator;
        typedef boost::shared_ptr<Graph>                                   GraphPtr;
        typedef boost::shared_ptr<const Graph>                             GraphConstPtr;
        typedef boost::reference_wrapper<Graph>                            GraphRef;


        /** Construct a random walker segmentation object.
          *
          * Random walker segmentation has a by-product: a matrix of
          * vertex/seed potentials. This is of no interest for a general user,
          * however those who are actually interested in these data may access
          * it after running segmentation with getPotentials(). Assembly and
          * storage if this matrix might be expensive if the graph is big, thus
          * this feature is disable by default and may be re-enabled on demand
          * with the \a store_potentials parameter.
          *
          * \param[in] store_potentials controls whether the vertex potentials
          * matrix should be assembled and stored during segmentation */
        RandomWalkerSegmentation (bool store_potentials = false);


        /** Destructor. */
        virtual
        ~RandomWalkerSegmentation ();


        /** Provide a pointer to the input point cloud.
          *
          * The graph representation for the input data will be computed using
          * the default graph builder and edge weight computer. */
        virtual void
        setInputCloud (const PointCloudConstPtr &cloud);


        /** Provide a pointer to the input point cloud graph. */
        void
        setInputGraph (const GraphPtr& graph);


        /** Provide a pointer to the cloud of seeds for random walker.
          *
          * A seed is a point with XYZ coordinates and a label. Seeds need not
          * exactly coincide with any point in the input data. A kD-tree will
          * be used to find the closest neighbor for each seed.
          *
          * There may be multiple seeds with the same label, they will be
          * considered to belong to the same cluster.
          *
          * \warn Label \c 0xFFFFFFFF is not allowed! */
        void
        setSeeds (const pcl::PointCloud<PointXYZL>::ConstPtr& seeds);


        /** Perform random walker segmentation.
          *
          * The clusters in the output vector will be in the order of inceasing
          * label of their corresponding seeds.
          *
          * \note During the segmentation the `vertex_color_t` property of the
          * graph is modified to reflect the computed segmentation.
          *
          * \param[out] clusters the resultant set of indices, indexing the
          * points of the input cloud or input graph that correspond to the
          * clusters */
        void
        segment (std::vector<pcl::PointIndices>& clusters);


        /** Build a graph based on the input dataset.
          *
          * This function constructs a graph and assigns edge weights. The user
          * has to call this function explicitly only if he needs to access the
          * constructed graph before running segmentation (e.g. to select seeds
          * among graph vertices).
          *
          * Example usage:
          *
          * ~~~{.cpp}
          * RandomWalkerSegmentation<PointT> rws;
          * rws.setInputCloud (cloud);
          * rws.preComputeGraph ();
          * auto graph = rws.getGraph ();
          * // Visualize graph or graph point cloud
          * // Ask the user to select seeds
          * rws.setSeeds (seeds);
          * rws.segment (clusters);
          * ~~~
          *
          * \note It is not necessary to call this function explicitly before
          * performing segmentation with segment()! */
        void
        preComputeGraph ();


        /** Returns the graph that was built (or provided with setInputGraph())
          * to perform random walker segmentation. */
        inline GraphConstPtr
        getGraph () const
        {
          return (graph_);
        }


        /** Get vertex potentials computed during random walker segmentation.
          *
          * This returns a const reference to a matrix of potentials formed
          * during segmentation and stored inside this object.
          *
          * \note Matrix assembly should be enabled during construction of the
          * segmentation object. If it was no enabled then zero-sized matrix
          * will be returned. */
        const Eigen::MatrixXf&
        getPotentials () const;

      private:

        typedef
          typename boost::property_map<
            Graph
          , boost::vertex_color_t
          >::type
        VertexColorMap;

        bool input_as_cloud_;

        GraphPtr graph_;
        std::vector<GraphRef> graph_components_;

        pcl::PointCloud<PointXYZL>::ConstPtr seeds_;

        pcl::graph::VoxelGridGraphBuilder<PointT, Graph> graph_builder_;

        /// Maintains bi-directional mapping between seed labels and color
        /// identifiers (which are used in random walker segmentation).
        boost::bimap<uint32_t, uint32_t> label_color_bimap_;

        bool store_potentials_;
        Eigen::MatrixXf potentials_;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };

  }

}

#include "impl/random_walker_segmentation.hpp"

#endif /* PCL_SEGMENTATION_RANDOM_WALKER_SEGMENTATION_H */
