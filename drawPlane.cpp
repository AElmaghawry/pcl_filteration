// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/filters/extract_indices.h>

// int main()
// {
//     // Load point cloud data from file
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::io::loadPCDFile<pcl::PointXYZ>("/home/joe/pcl_filteration/0.pcd", *cloud);

//     // Estimate surface normals
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//     ne.setInputCloud(cloud);
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//     ne.setSearchMethod(tree);
//     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//     ne.setRadiusSearch(0.03); // Adjust the radius as needed
//     ne.compute(*normals);

//     // Select a reference point on the surface
//     int referenceIndex = 100; // Modify this index according to your needs

//     // Compute the plane coefficients
//     Eigen::Vector4f planeCoefficients;
//     planeCoefficients[0] = normals->points[referenceIndex].normal_x;
//     planeCoefficients[1] = normals->points[referenceIndex].normal_y;
//     planeCoefficients[2] = normals->points[referenceIndex].normal_z;
//     planeCoefficients[3] = -(
//         planeCoefficients[0] * cloud->points[referenceIndex].x +
//         planeCoefficients[1] * cloud->points[referenceIndex].y +
//         planeCoefficients[2] * cloud->points[referenceIndex].z
//     );

//     // Create a new point cloud representing the plane
//     pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
//     for (float x = -1.0f; x <= 1.0f; x += 0.01f) {
//         for (float y = -1.0f; y <= 1.0f; y += 0.01f) {
//             pcl::PointXYZ point;
//             point.x = x;
//             point.y = y;
//             point.z = -(planeCoefficients[0] * x + planeCoefficients[1] * y + planeCoefficients[3]) / planeCoefficients[2];
//             plane->points.push_back(point);
//         }
//     }
//     plane->width = plane->points.size();
//     plane->height = 1;
//     plane->is_dense = true;

//     // Save the plane as a new point cloud
//     pcl::io::savePCDFileASCII("parallel_plane.pcd", *plane);

//     std::cout << "Parallel plane extracted and saved as parallel_plane.pcd" << std::endl;

//     return 0;
// }

// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>

// int main()
// {
//     // Load point cloud data from file
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::io::loadPCDFile<pcl::PointXYZ>("/home/joe/pcl_filteration/0.pcd", *cloud);

//     // Get the total number of points in the point cloud
//     std::size_t numPoints = cloud->points.size();

//     // Select a random reference index
//     std::size_t referenceIndex = std::rand() % numPoints;

//     // Print the chosen reference index
//     std::cout << "Reference index: " << referenceIndex << std::endl;

//     return 0;
// }
// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/features/shot.h>
// #include <pcl/keypoints/sift_keypoint.h>

// int main()
// {
//     // Load point cloud data from file
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::io::loadPCDFile<pcl::PointXYZ>("/home/joe/pcl_filteration/0.pcd", *cloud);

//     // Estimate surface normals
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//     ne.setInputCloud(cloud);
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//     ne.setSearchMethod(tree);
//     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//     ne.setRadiusSearch(0.03);
//     ne.compute(*normals);

//     // Create the SIFTKeypoint object
//     pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointWithScale> sift;
//     sift.setInputCloud(cloud);
//     sift.setSearchSurface(cloud);
//     sift.setInputNormals(normals);
//     sift.setScales(0.01, 3, 5);
//     sift.setMinimumContrast(0.01);

//     // Compute keypoints
//     pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints(new pcl::PointCloud<pcl::PointWithScale>);
//     sift.compute(*keypoints);

//     // Select a reference index based on your criteria
//     int referenceIndex = 0; // Use the first keypoint as the reference

//     // Print the chosen reference index
//     std::cout << "Reference index: " << referenceIndex << std::endl;

//     return 0;
// 
// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/features/normal_3d.h>
// #include <cmath>

// void calculateRollPitchYaw(float nx, float ny, float nz, float& roll, float& pitch, float& yaw) {
//   // Roll (rotation around x-axis)
//   roll = std::atan2(ny, nz);

//   // Pitch (rotation around y-axis)
//   pitch = std::atan2(-nx, std::sqrt(ny * ny + nz * nz));

//   // Yaw (rotation around z-axis)
//   yaw = 0.0f;  // Yaw is not well-defined for a single direction vector
// }

// int main() {
//   // Load point cloud data from file
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//   if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/joe/pcl_filteration/test.pcd", *cloud) == -1) {
//     PCL_ERROR("Couldn't read the input point cloud file.\n");
//     return -1;
//   }

//   // Create the normal estimation object
//   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//   ne.setInputCloud(cloud);

//   // Create an empty kdtree representation
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//   ne.setSearchMethod(tree);

//   // Set the search radius for determining the neighborhood size
//   ne.setRadiusSearch(0.03);  // Adjust this value based on your point cloud density

//   // Compute the normals
//   pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//   ne.compute(*normals);

//   // Compute the average direction vector
//   float avg_x = 0.0f;
//   float avg_y = 0.0f;
//   float avg_z = 0.0f;
//   for (size_t i = 0; i < normals->size(); ++i) {
//     const float nx = normals->points[i].normal_x;
//     const float ny = normals->points[i].normal_y;
//     const float nz = normals->points[i].normal_z;

//     avg_x += nx;
//     avg_y += ny;
//     avg_z += nz;
//   }
//   const float avg_magnitude = std::sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z);
//   avg_x /= avg_magnitude;
//   avg_y /= avg_magnitude;
//   avg_z /= avg_magnitude;

//   // Calculate the resultant angle and magnitude
//   const float resultant_magnitude = avg_magnitude / normals->size();
//   const float resultant_angle = std::acos(avg_z) * 180.0 / M_PI;

//   // Calculate roll, pitch, and yaw angles
//   float roll, pitch, yaw;
//   calculateRollPitchYaw(avg_x, avg_y, avg_z, roll, pitch, yaw);

//   std::cout << "Resultant Normal:" << std::endl;
//   std::cout << "  Magnitude: " << resultant_magnitude << std::endl;
//   std::cout << "  Angle: " << resultant_angle << " degrees" << std::endl;
//   std::cout << "  Roll: " << roll << " radians" << std::endl;
//   std::cout << "  Pitch: " << pitch << " radians" << std::endl;
//   std::cout << "  Yaw: " << yaw << " radians" << std::endl;

//   return 0;
// }
// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <Eigen/Core>
// #include <Eigen/Dense>

// typedef pcl::PointXYZ PointT;
// typedef pcl::PointCloud<PointT> PointCloudT;

// int main()
// {
//     // Load the point cloud data
//     PointCloudT::Ptr cloud(new PointCloudT);
//     pcl::io::loadPCDFile<PointT>("/home/joe/pcl_filteration/0.pcd", *cloud);

//     // Prepare the data for curve fitting
//     Eigen::MatrixXd A(cloud->size(), 2);
//     Eigen::VectorXd b(cloud->size());

//     for (size_t i = 0; i < cloud->size(); ++i)
//     {
//         A(i, 0) = (*cloud)[i].x;
//         A(i, 1) = 1.0;
//         b(i) = (*cloud)[i].y;
//     }

//     // Perform least squares fitting
//     Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

//     // Retrieve the fitted curve parameters
//     double slope = x(0);
//     double intercept = x(1);

//     // Display the line equation (y = mx + c)
//     std::cout << "Fitted line equation: y = " << slope << "x + " << intercept << std::endl;

//     return 0;
// }

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

int
 main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 15;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate the data
  for (auto& point: *cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1.0;
  }

  // Set a few outliers
  (*cloud)[0].z = 2.0;
  (*cloud)[3].z = -2.0;
  (*cloud)[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (const auto& idx: inliers->indices)
    std::cerr << idx << "    " << cloud->points[idx].x << " "
                               << cloud->points[idx].y << " "
                               << cloud->points[idx].z << std::endl;

  return (0);
}