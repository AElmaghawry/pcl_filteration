#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>


int main()
{
  // Load the point cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/joe/pointCloud/0.pcd", *cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Compute the center point of the point cloud
  pcl::PointXYZ center;
  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for (const auto& point : cloud->points) {
    centroid.add(point);
  }
  centroid.get(center);

  // Estimate the plane passing through the center point
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  // Generate a planar representation of the plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (const auto& index : inliers->indices) {
    plane->push_back((*cloud)[index]);
    pcl::PointXYZRGB colored_point;
    colored_point.x = (*cloud)[index].x;
    colored_point.y = (*cloud)[index].y;
    colored_point.z = (*cloud)[index].z;
    colored_point.r = 255;
    colored_point.g = 0;
    colored_point.b = 0;
    colored_cloud->push_back(colored_point);
  }

  // Create a new point cloud for the centroid
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB centroid_point;
  centroid_point.x = center.x;
  centroid_point.y = center.y;
  centroid_point.z = center.z;
  centroid_point.r = 0;
  centroid_point.g = 255;
  centroid_point.b = 0;
  centroid_cloud->push_back(centroid_point);

  // Create points for the axes
  pcl::PointXYZRGB x_axis, y_axis, z_axis;
  x_axis.x = center.x + 1.0; // Length of the x-axis line
  x_axis.y = center.y;
  x_axis.z = center.z;
  x_axis.r = 255;
  x_axis.g = 0;
  x_axis.b = 0;

  y_axis.x = center.x;
  y_axis.y = center.y + 1.0; // Length of the y-axis line
  y_axis.z = center.z;
  y_axis.r = 0;
  y_axis.g = 255;
  y_axis.b = 0;

  z_axis.x = center.x;
  z_axis.y = center.y;
  z_axis.z = center.z + 1.0; // Length of the z-axis line
  z_axis.r = 0;
  z_axis.g = 0;
  z_axis.b = 255;

  // Visualize the plane, centroid, and axes
  pcl::visualization::PCLVisualizer viewer("Plane Viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.0);
  // viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud, "cloud");
  // viewer.addPointCloud<pcl::PointXYZRGB>(centroid_cloud, "centroid");
  // viewer.addLine<pcl::PointXYZRGB>(centroid_point, x_axis, "x_axis");
  // viewer.addLine<pcl::PointXYZRGB>(centroid_point, y_axis, "y_axis");
  // viewer.addLine<pcl::PointXYZRGB>(centroid_point, z_axis, "z_axis");
  // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
  // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "centroid");
  viewer.addCoordinateSystem (1.0, "global");
  // viewer.addCoordinateSystem(1.0, centroid_point.x,centroid_point.y,centroid_point.z);
  
      std::cerr << "Cloud after filtering: " << std::endl;
  for (const auto& point: *centroid_cloud)
    std::cerr << "    " << centroid_point.x << " "
                        << centroid_point.y << " "
                        << centroid_point.z << std::endl;
                        
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(1.1470, 1.2); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter(*filtered_cloud);

    // Create a PCL visualizer object
  // pcl::visualization::PCLVisualizer viewer("Filtered Point Cloud");

    // Add the filtered point cloud to the visualizer
  viewer.addPointCloud<pcl::PointXYZ>(filtered_cloud, "cloud");


  viewer.spin();

  return 0;
}
