#include <iostream>
#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>

double calcAngelTwoVectors(pcl::ModelCoefficients::Ptr coefficients1 ,pcl::ModelCoefficients::Ptr coefficients2)
{
    double dot {} ; 
    double lenSq1 {} ; 
    double lenSq2 {}; 
    double angle {}; 

    double x1,x2,y1,y2,z1,z2{}; 

    x1= coefficients1->values[0]; 
    y1= coefficients1->values[1];
    z1= coefficients1->values[2];
    x2= coefficients2->values[0];
    y2= coefficients2->values[1];
    z2= coefficients2->values[2];

    dot = x1*x2 + y1*y2 + z1*z2 ; 
    lenSq1 = x1*x1 + y1*y1 + z1*z1;
    lenSq2 = x2*x2 + y2*y2 + z2*z2;
    angle = acos(dot/sqrt(lenSq1 * lenSq2));
    angle = angle * 180 / 3.14 ; 
    return angle ; 
}
int main (int argc, char **argv){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("../data/cloud_cluster_0.pcd", *cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::PointXYZRGB centroid;
    pcl::computeCentroid(*cloud, centroid);

    for (pcl::PointXYZRGB& point : cloud->points)
    {
        point.x -= centroid.x;
        point.y -= centroid.y;
        point.z -= centroid.z;
    }
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;
                                        

    pcl::ModelCoefficients::Ptr coefficientsRef (new pcl::ModelCoefficients);
    coefficientsRef->values.push_back(0.0); 
    coefficientsRef->values.push_back(0.0); 
    coefficientsRef->values.push_back(1.0); 
    coefficientsRef->values.push_back(0.0);   


    pcl::visualization::PCLVisualizer viewer("Point Cloud Visualization");
    
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
    viewer.addPointCloud(cloud, "point_cloud");
    
    viewer.addPlane(*coefficients,"plane");
    viewer.addPlane(*coefficientsRef,"refrence"); 

    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1.0,1.0,0.0,"plane"); 
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,1.0,1.0,"refrence"); 

    std::cout << "________________________________________"<<std::endl ; 

    std::cout   << "Measured Angle between refrence frame and Surface is = "
                << calcAngelTwoVectors(coefficientsRef,coefficients)
                << " degrees"
                <<std::endl ; 
                
    viewer.addCoordinateSystem(0.1);
    viewer.spin(); 

    return 0; 

}