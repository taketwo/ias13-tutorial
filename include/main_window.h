#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "seed_selection.h"
#include "random_walker_segmentation.h"

namespace Ui
{
  class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:

    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointXYZRGBNormal PointWithNormalT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef pcl::PointCloud<PointWithNormalT> PointCloudWithNormalT;

    typedef pcl::segmentation::RandomWalkerSegmentation<pcl::PointXYZRGB> RandomWalkerSegmentation;
    typedef RandomWalkerSegmentation::Graph Graph;
    typedef RandomWalkerSegmentation::GraphPtr GraphPtr;
    typedef RandomWalkerSegmentation::GraphConstPtr GraphConstPtr;

    MainWindow (const std::string& filename, QWidget* parent = 0);

    ~MainWindow();

  public Q_SLOTS:

    void
    buttonUpdateClicked ();

    void
    buttonNewLabelClicked ();

    void
    buttonDeleteLabelClicked ();

    void
    buttonSegmentClicked ();

    void
    seedsChanged ();

  private:

    void
    pointPickingCallback (const pcl::visualization::PointPickingEvent& event, void*);

    void
    displayGraphVertices (bool how = true);

    void
    displaySeeds ();

    Ui::MainWindow* ui_;

    pcl::visualization::PCLVisualizer::Ptr viewer_;

    PointCloudT::Ptr cloud_;
    GraphPtr graph_;
    SeedSelection::Ptr seed_selection_;

};

#endif // MAIN_WINDOW_H
