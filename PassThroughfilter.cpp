#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    // Load the input point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/joe/pcl_filteration/0.pcd", *cloud);

    // Apply the pass-through filter along the Z-axis
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(1.1470, 1.2); // Example filter limits
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*filtered_cloud);

    // Create a PCL visualizer object
    pcl::visualization::PCLVisualizer viewer("Filtered Point Cloud");

    // Add the filtered point cloud to the visualizer
    viewer.addPointCloud<pcl::PointXYZ>(filtered_cloud, "cloud");

    // Set the visualizer background color
    viewer.setBackgroundColor(0.0, 0.0, 0.0);  // Black background

    // Set the visualizer camera position and orientation
    // viewer.setCameraPosition(0.0, 0.0, -2.0, 0.0, -1.0, 0.0);
    pcl::io::savePCDFileASCII("test.pcd", *filtered_cloud);
    // Start the visualizer main loop
    // while (!viewer.wasStopped())
    {
        viewer.spin();
    }

    return 0;
}
