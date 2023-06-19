#include <list>
#include <math.h>
#include <vector>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/don.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/search/organized.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <unsupported/Eigen/EulerAngles>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>


double calcAngelTwoVectors(pcl::ModelCoefficients::Ptr coefficients1 ,pcl::ModelCoefficients::Ptr coefficients2);


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

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */