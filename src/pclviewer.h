#pragma once

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

public Q_SLOTS:
  void
  startSlot ();

  void
  pcdBrowseSlot ();

  void
  classBrowseSlot ();

  void
  pointSpinBoxValueChanged (int value);

  void
  ratioSpinBoxValueChanged (double value);

  void
  angleSpinBoxValueChanged (double value);

  void
  lengthSpinBoxValueChanged (double value);

  void
  triangulation();

  void
  mesh();

protected:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  PointCloudT::Ptr cloud;

  double length_ratio_;
  double max_angle_;
  double max_length_;

  std::vector<int> triangles_;
private:
  Ui::PCLViewer *ui;
};
