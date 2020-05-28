#include "pclviewer.h"
#include "ui_pclviewer.h"
#include "QFileDialog"

#include <thread>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <string>
#include <math.h>
#include "mesher.h"


PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);

  // The default color
  length_ratio_ = ui->ratioSpinBox->value();
  max_angle_ = ui->angleSpinBox->value();
  max_length_ = ui->lengthSpinBox->value();

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->visualizer->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->visualizer->GetInteractor (), ui->visualizer->GetRenderWindow ());
  ui->visualizer->update ();

  // Connect "START" button and the function
  //connect (ui->start_button,  SIGNAL (clicked ()), this, SLOT (startSlot ()));

  //connect (ui->path_to_pcd, SIGNAL (clicked ()), this, SLOT (pcdBrowseSlot ()));
  //connect (ui->path_to_class, SIGNAL (clicked ()), this, SLOT (classBrowseSlot ()));

  // Connect spin boxes to their functions
  //connect (ui->ratioSpinBox, SIGNAL (valueChanged (double)), this, SLOT (ratioSpinBoxValueChanged (double)));
  //connect (ui->angleSpinBox, SIGNAL (valueChanged (double)), this, SLOT (angleSpinBoxValueChanged (double)));
  //connect (ui->lengthSpinBox, SIGNAL (valueChanged (double)), this, SLOT (lengthSpinBoxValueChanged (double)));
  

  viewer->setBackgroundColor(.7, .7, .7);
  viewer->resetCamera ();
  ui->visualizer->update ();
}

void
PCLViewer::startSlot ()
{
  printf ("Start button was pressed\n");

  std::string pcd_file = ui->path_to_pcd->text().toStdString();
  std::string class_file = ui->path_to_class->text().toStdString();

  cloud.reset (new PointCloudT);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file, *cloud);
  cv::Mat classification = cv::imread(class_file);
  std::cout << "Loaded " << cloud->width * cloud->height << " data points from data.pcd with the following fields: " << std::endl;

  for (auto i = 0; i < cloud->points.size(); i++) {
	double x = cloud->points[i].x;
	double y = cloud->points[i].y;
	double z = cloud->points[i].z;

	z = 1.f / z;
	int col = fx * x * z + cx;
	int row = fy * y * z + cy;

	auto index = classification.at<unsigned char>(row, col * 3);

	if (index > 39) continue;

	cloud->points[i].r = r_values[index];
	cloud->points[i].g = g_values[index];
	cloud->points[i].b = b_values[index];
  }

  viewer->removePointCloud("cloud");
  viewer->addPointCloud<PointT>(cloud, "cloud");
  //viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, -1, 0);
  triangulation();
  mesh();
}

void
PCLViewer::pcdBrowseSlot ()
{
  printf ("pcd button was pressed\n");
  QString fileName = QFileDialog::getOpenFileName( this, tr("Open PCD File"), "", tr("PCD File (*.pcd);;All Files (*)"));
  ui->path_to_pcd->setText(fileName);
}

void
PCLViewer::classBrowseSlot ()
{
  printf ("Class button was pressed\n");
  QString fileName = QFileDialog::getOpenFileName( this, tr("Open Classification File"), "", tr("Classification File (*.bmp);;All Files (*)"));
  ui->path_to_class->setText(fileName);
}

void
PCLViewer::pointSpinBoxValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  ui->visualizer->update ();
}

void
PCLViewer::ratioSpinBoxValueChanged (double value)
{
  length_ratio_ = value;
  printf ("ratioSpinBoxValueChanged: [%f|%f|%f]\n", length_ratio_, max_angle_, max_length_);
  mesh();
}

void
PCLViewer::angleSpinBoxValueChanged (double value)
{
  max_angle_ = value;
  printf ("angleSpinBoxValueChanged: [%f|%f|%f]\n", length_ratio_, max_angle_, max_length_);
  mesh();
}

void
PCLViewer::lengthSpinBoxValueChanged (double value)
{
  max_length_ = value;
  printf("lengthSpinBoxValueChanged: [%f|%f|%f]\n", length_ratio_, max_angle_, max_length_);
  mesh();
}

void
PCLViewer::triangulation ()
{
  Vec3_t cam;
  cam[0] = 0, cam[1] = 0, cam[2] = 0;
  mesher::Mesher m(cloud, cam);
  triangles_ = m.getTriangles();
}

void
PCLViewer::mesh ()
{
  std::vector<pcl::Vertices> polys;
  pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);

  for (auto j = 0; j < triangles_.size(); j += 3) {
	pcl::PointXYZRGB tri1, tri2, tri3;
	tri1 = cloud->points[triangles_[j]];
	tri2 = cloud->points[triangles_[j + 1]];
	tri3 = cloud->points[triangles_[j + 2]];

	if (!mesher::isValid(tri1.x, tri1.y, tri1.z, tri2.x, tri2.y, tri2.z, tri3.x, tri3.y, tri3.z, length_ratio_, max_angle_, max_length_))
		continue;


	pcl::Vertices v;
	v.vertices.push_back(triangles_[j]);
	v.vertices.push_back(triangles_[j + 1]);
	v.vertices.push_back(triangles_[j + 2]);

	polys.push_back(v);
  }

  pcl::PolygonMesh mesh;
  mesh.polygons = polys;
  pcl::toPCLPointCloud2(*cloud, *cloud_blob);
  mesh.cloud = *cloud_blob;
  viewer->removePolygonMesh("mesh");
  viewer->addPolygonMesh(mesh, "mesh");

  //viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, -1, 0);
  ui->visualizer->update();
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
