#include "seed_selection.h"

#include <pcl/common/utils.h>
#include <pcl/console/print.h>
#include <pcl/common/io.h>

SeedSelection::SeedSelection (QObject* parent)
: QAbstractListModel (parent)
, seeds_cloud_ (new LabeledPointCloud)
, num_labels_ (1)
, current_label_ (1)
{
}

SeedSelection::~SeedSelection ()
{
}

int
SeedSelection::rowCount (const QModelIndex&) const
{
  return num_labels_;
}

QVariant
SeedSelection::data (const QModelIndex& index, int role) const
{
  if (role == Qt::DisplayRole)
    return QString ("Label %1").arg (index.row() + 1);
  return QVariant ();
}

void
SeedSelection::pointPicked (const pcl::PointXYZ& p)
{
  if (isDuplicate (p))
    return;

  pcl::PointXYZL pt;
  pt.x = p.x, pt.y = p.y, pt.z = p.z, pt.label = current_label_;
  seeds_cloud_->push_back (pt);
}

QModelIndex
SeedSelection::addNewLabel ()
{
  ++num_labels_;
  current_label_ = num_labels_;
  dataChanged (QModelIndex (), QModelIndex ());
  return index (current_label_ - 1, 0);
}

void
SeedSelection::deleteLabel ()
{
  if (num_labels_ == 1)
    return;

  LabeledPointCloudPtr new_seeds (new LabeledPointCloud);
  for (size_t i = 0; i < seeds_cloud_->size (); ++i)
    if (seeds_cloud_->at (i).label != current_label_)
    {
      new_seeds->push_back (seeds_cloud_->at (i));
      if (seeds_cloud_->at (i).label > current_label_)
        --new_seeds->back ().label;
    }

  --num_labels_;
  seeds_cloud_.swap (new_seeds);
  dataChanged (QModelIndex (), QModelIndex ());
  seedsChanged ();
}

SeedSelection::ColoredPointCloudPtr
SeedSelection::getPointCloudForVisualization ()
{
  ColoredPointCloudPtr cloud (new ColoredPointCloud);
  pcl::copyPointCloud (*seeds_cloud_, *cloud);
  for (size_t i = 0; i < cloud->size (); ++i)
    if (seeds_cloud_->at (i).label == current_label_)
      cloud->at (i).rgb = 0xFFFFFF;
    else
      cloud->at (i).rgb = 0xFF0000;
  return cloud;
}

void
SeedSelection::currentChanged (const QItemSelection& current, const QItemSelection& previous)
{
  current_label_ = current.indexes ().back ().row () + 1;
  seedsChanged ();
}

bool
SeedSelection::isDuplicate (const pcl::PointXYZ& pt)
{
  using namespace pcl::utils;
  for (size_t i = 0; i < seeds_cloud_->size (); ++i)
  {
    const pcl::PointXYZL& p = seeds_cloud_->points[i];
    if (equal (p.x, pt.x) && equal (p.y, pt.y) && equal (p.z, pt.z))
      return true;
  }
  return false;
}

