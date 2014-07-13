#ifndef SEED_SELECTION_H
#define SEED_SELECTION_H

#include <QAbstractListModel>
#include <QItemSelection>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class SeedSelection : public QAbstractListModel
{

    Q_OBJECT

  public:

    typedef boost::shared_ptr<SeedSelection> Ptr;

    typedef pcl::PointCloud<pcl::PointXYZRGB> ColoredPointCloud;
    typedef ColoredPointCloud::Ptr ColoredPointCloudPtr;

    typedef pcl::PointCloud<pcl::PointXYZL> LabeledPointCloud;
    typedef LabeledPointCloud::Ptr LabeledPointCloudPtr;

    SeedSelection (QObject* parent = 0);

    ~SeedSelection ();

    virtual int
    rowCount (const QModelIndex&) const;

    virtual QVariant
    data (const QModelIndex& index, int role) const;

    void
    pointPicked (const pcl::PointXYZ& p);

    QModelIndex
    addNewLabel ();

    void
    deleteLabel ();

    LabeledPointCloudPtr
    getSelectedSeeds ()
    {
      return seeds_cloud_;
    }

    ColoredPointCloudPtr
    getPointCloudForVisualization ();

  public Q_SLOTS:

    void
    currentChanged (const QItemSelection& current, const QItemSelection& previous);

  Q_SIGNALS:

    void
    seedsChanged ();

  private:

    bool isDuplicate (const pcl::PointXYZ& pt);

    LabeledPointCloudPtr seeds_cloud_;

    size_t num_labels_;

    uint32_t current_label_;

};

#endif /* SEED_SELECTION_H */

