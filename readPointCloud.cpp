#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main() {
    // Load the point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrginal(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/joe/pcl_filteration/0.pcd", *cloudOrginal);

    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/joe/pcl_filteration/cloud_cluster_0.pcd", *cloud1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/joe/pcl_filteration/cloud_cluster_1.pcd", *cloud2);

    // Initialize the visualizer
    pcl::visualization::PCLVisualizer viewer0("Point Cloud Visualization");
    pcl::visualization::PCLVisualizer viewer1("Point Cloud Visualization1");
    pcl::visualization::PCLVisualizer viewer2("Point Cloud Visualization2");

    // Add the point cloud to the viewer
    viewer0.addPointCloud(cloudOrginal, "point_cloud_original");
    viewer1.addPointCloud(cloud1, "point_cloud_1");
    viewer2.addPointCloud(cloud2, "point_cloud_2");

    // Customize the visualization (optional)
    viewer0.setSize(640,480);
    viewer1.setSize(640,480); 
    viewer2.setSize(640,480);  
    viewer0.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point_cloud_original");
    viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point_cloud_1");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point_cloud_2");


    // Start the visualization loop
    viewer0.spin();
    viewer1.spin();
    viewer2.spin();
    return 0;
}
