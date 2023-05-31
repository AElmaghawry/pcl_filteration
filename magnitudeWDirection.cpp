#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <cmath>

void calculateRollPitchYaw(float nx, float ny, float nz, float& roll, float& pitch, float& yaw) {
  // Roll (rotation around x-axis)
  roll = std::atan2(ny, nz);

  // Pitch (rotation around y-axis)
  pitch = std::atan2(-nx, std::sqrt(ny * ny + nz * nz));

  // Yaw (rotation around z-axis)
  yaw = 0.0f;  // Yaw is not well-defined for a single direction vector
}

int main() {
  // Load point cloud data from file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/joe/pcl_filteration/test.pcd", *cloud) == -1) {
    PCL_ERROR("Couldn't read the input point cloud file.\n");
    return -1;
  }

  // Create the normal estimation object
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  // Create an empty kdtree representation
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(tree);

  // Set the search radius for determining the neighborhood size
  ne.setRadiusSearch(0.1);  // Adjust this value based on your point cloud density

  // Compute the normals
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  // Compute the average direction vector
  float avg_x = 0.0f;
  float avg_y = 0.0f;
  float avg_z = 0.0f;
  for (size_t i = 0; i < normals->size(); ++i) {
    const float nx = normals->points[i].normal_x;
    const float ny = normals->points[i].normal_y;
    const float nz = normals->points[i].normal_z;

    avg_x += nx;
    avg_y += ny;
    avg_z += nz;
  }
  const float avg_magnitude = std::sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z);
  avg_x /= avg_magnitude;
  avg_y /= avg_magnitude;
  avg_z /= avg_magnitude;

  // Calculate the resultant angle and magnitude
  const float resultant_magnitude = avg_magnitude / normals->size();
  const float resultant_angle = std::acos(avg_z) * 180.0 / M_PI;

  // Calculate roll, pitch, and yaw angles
  float roll, pitch, yaw;
  calculateRollPitchYaw(avg_x, avg_y, avg_z, roll, pitch, yaw);

  std::cout << "Resultant Normal:" << std::endl;
  std::cout << "  Magnitude: " << resultant_magnitude << std::endl;
  std::cout << "  Angle: " << resultant_angle << " degrees" << std::endl;
  std::cout << "  Roll: " << roll << " radians" << std::endl;
  std::cout << "  Pitch: " << pitch << " radians" << std::endl;
  std::cout << "  Yaw: " << yaw << " radians" << std::endl;

  return 0;
}
