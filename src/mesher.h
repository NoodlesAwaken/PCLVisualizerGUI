#include <math.h>
#include <pcl/point_types.h>

#include "color.h"
#include "delaunator.h"

// hard code the camera parameters
static const double fx = 518.8579;
static const double fy = 519.4696;
static const double cx = 325.5824;
static const double cy = 253.7361;

using Vec3_t = Eigen::Vector3d;

namespace mesher {

inline bool isValid(double x1, double y1, double z1, 
					double x2, double y2, double z2, 
					double x3, double y3, double z3,
					double r, double a, double l) {
	// three sides of the triangle
	double l1 = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
	double l2 = sqrt(pow(x1 - x3, 2) + pow(y1 - y3, 2) + pow(z1 - z3, 2));
	double l3 = sqrt(pow(x3 - x2, 2) + pow(y3 - y2, 2) + pow(z3 - z2, 2));

	// three angles of the triangle
	double a1 = acos((l1 * l1 + l2 * l2 - l3 * l3) / (2 * l1 * l2)) * 180 / M_PI;
	double a2 = acos((l1 * l1 + l3 * l3 - l2 * l2) / (2 * l1 * l3)) * 180 / M_PI;
	double a3 = acos((l2 * l2 + l3 * l3 - l1 * l1) / (2 * l2 * l3)) * 180 / M_PI;

	//std::cout << a1 << " " << a2 << " " << a3 << "\n";

	if ((l1 / l2 > r && l3 / l2 > r) ||
		(l1 / l3 > r && l2 / l3 > r) ||
		(l2 / l1 > r && l3 / l1 > r) ||
		a1 > a || a2 > a || a3 > a ||
		l1 > l || l2 > l || l3 > l 	)
		return false;

	return true;
};

// get triangle index from input 2d coords
std::vector<int> get_triangles(std::vector<double> const& in_coords) {
	delaunator::Delaunator d(in_coords);
	std::vector<int> tri_index;

	for (auto i = 0; i < d.triangles.size(); i += 3) {
		double tx1 = d.coords[2 * d.triangles[i]];
		double ty1 = d.coords[2 * d.triangles[i] + 1];
		double tx2 = d.coords[2 * d.triangles[i + 1]];
		double ty2 = d.coords[2 * d.triangles[i + 1] + 1];
		double tx3 = d.coords[2 * d.triangles[i + 2]];
		double ty3 = d.coords[2 * d.triangles[i + 2] + 1];

		int index1 = 0, index2 = 0, index3 = 0;
		for (auto j = 0; j < in_coords.size(); j += 2) {
			if (in_coords[j] == tx1 && in_coords[j + 1] == ty1)
				index1 = j;
			else if (in_coords[j] == tx2 && in_coords[j + 1] == ty2)
				index2 = j;
			else if (in_coords[j] == tx3 && in_coords[j + 1] == ty3)
				index3 = j;
		}

		tri_index.push_back(index1 / 2);
		tri_index.push_back(index2 / 2);
		tri_index.push_back(index3 / 2);
	}
	
	return tri_index;
};

// back project 3D points to 2D pixel location
std::vector<double> get2dcoords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr const& cloud, Vec3_t cam) {
	std::vector<double> coords;
	for (auto i = 0; i < cloud->points.size(); i++) {
		double x = cloud->points[i].x - cam[0],
				y = cloud->points[i].y - cam[1],
				z = cloud->points[i].z - cam[2];

		double z_inv = 1.0/z;
		double coorx = x * fx * z_inv + cx;
		double coory = y * fy * z_inv + cy;

		coords.push_back(coorx);
		coords.push_back(coory);
	}

	return coords;
};

class Mesher {
public:
	Mesher(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Vec3_t cam);

	std::vector<int> getTriangles();
private:
	std::vector<int> triangles_;
};

Mesher::Mesher(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Vec3_t cam) {
	triangles_ = get_triangles(get2dcoords(cloud, cam));
}

std::vector<int> Mesher::getTriangles() {
	return triangles_;
}

}
