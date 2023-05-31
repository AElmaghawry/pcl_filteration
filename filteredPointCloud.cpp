#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
  // Load the point cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/joe/pcl_filteration/0.pcd", *cloud);

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

  // Visualize the plane and centroid
  pcl::visualization::PCLVisualizer viewer("Plane Viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.0);
  viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud, "cloud");
  viewer.addPointCloud<pcl::PointXYZRGB>(centroid_cloud, "centroid");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "centroid");
  
    std::cerr << "Cloud after filtering: " << std::endl;
  for (const auto& point: *centroid_cloud)
    std::cerr << "    " << centroid_point.x << " "
                        << centroid_point.y << " "
                        << centroid_point.z << std::endl;
                        
                        
  viewer.spin();

  return 0;
}
