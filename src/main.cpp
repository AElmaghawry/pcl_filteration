#include <bladAngleDetection.hpp>

class PointCloudProcessing
{
public:
    bool loadPointCloud(const std::string &file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read the file \n");
            return false;
        }
        std::cout << "Loaded "
                  << cloud->width * cloud->height
                  << " :"
                  << file_name
                  << std::endl;
        return true;
    }

    void saveClusterPointClouds(const std::vector<pcl::PointIndices> &clusters, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud, int minClusterSize, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters_vector)
    {
        pcl::PCDWriter writer;
        int j = 0;
        for (const auto &indices : clusters)
        {
            if (indices.indices.size() >= minClusterSize)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
                for (const auto &pt : indices.indices)
                {
                    cloud_cluster->push_back((*colored_cloud)[pt]);
                }
                cloud_cluster->width = indices.indices.size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                
                clusters_vector.push_back(cloud_cluster);

                std::stringstream ss;
                ss << "../data/cloud_cluster_" << j << ".pcd";
                writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_cluster, false);
                j++;
            }
        }
    }

    void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals)
    {
        pcl::visualization::PCLVisualizer viewer("Cluster viewer");
        viewer.addPointCloud(colored_cloud, "cluster viewer");
        viewer.setBackgroundColor(0.0, 0.0, 0.0);
        viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(colored_cloud, normals);
        viewer.spinOnce();
    }
};

class RegionGrowingSegmentation
{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    std::vector<pcl::PointIndices> clusters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

public:
    void performSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
    {
        cloud = input_cloud;

        tree.reset(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        normals.reset(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;

        normal_estimator.setInputCloud(cloud);
        normal_estimator.setSearchMethod(tree);

        normal_estimator.setKSearch(150);
        normal_estimator.compute(*normals);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(1000);
        reg.setMaxClusterSize(1000000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(50);
        reg.setInputCloud(cloud);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold(0.01);

        reg.extract(clusters);
        std::vector<pcl::PointIndices> blade_cluster{};
        for (int i{0}; i < clusters.size(); ++i)
        {

            blade_cluster.push_back(clusters[i]);
            std::cout << "Cluster " << i + 1
                      << " has "
                      << clusters[i].indices.size()
                      << " points." << std::endl;
        }
        colored_cloud = reg.getColoredCloud();
    }

    const std::vector<pcl::PointIndices> &getClusters() const
    {
        return clusters;
    }

    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &getColoredCloud() const
    {
        return colored_cloud;
    }

    const pcl::PointCloud<pcl::Normal>::Ptr &getNormals() const
    {
        return normals;
    }
};

int main(int argc, char **argv)
{
    std::string file_name = "../data/data_set/4/4.pcd";
    
    PointCloudProcessing pcProcessing;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!pcProcessing.loadPointCloud(file_name, cloud))
        return -1;

    RegionGrowingSegmentation regionSegmentation;
    regionSegmentation.performSegmentation(cloud);

    const std::vector<pcl::PointIndices> &clusters = regionSegmentation.getClusters();
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud = regionSegmentation.getColoredCloud();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_vector;
    pcProcessing.saveClusterPointClouds(clusters, colored_cloud, 1500, clusters_vector);

    pcProcessing.visualizePointCloud(colored_cloud, regionSegmentation.getNormals());

    std::cout << clusters.size() << std::endl;

    std::string cloud_of_intreset_dir = "../data/cloud_cluster_"; ///home/jeo/ku/pcl_filteration/data/cloud_cluster_0.pcd

    std::vector<std::string> files_paths{};
    PointCloudProcessing pcSelection;
    
    for (int i{0}; i < clusters.size(); i++)
    {
        files_paths.push_back(cloud_of_intreset_dir + std::to_string(i) + ".pcd");
    }
    std::cout << files_paths[1] << std::endl;

    std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointClouds; 
    int i {0} ; 
    for (const std::string &filepath : files_paths)
    {   
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile<pcl::PointXYZRGB>(filepath, *cloud);
        pointClouds.push_back(cloud);
        i++ ;
        std::cout<< "Loading the pcd files ......" << i << std::endl ; 

    }

    size_t smallestSize = std::numeric_limits<size_t>::max();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr smallestCloud;

    for (const auto& cloud : clusters_vector)
    {
        size_t cloudSize = cloud->size();
        if (cloudSize < smallestSize)
        {
            smallestSize = cloudSize;
            smallestCloud = cloud;
        }
    }

    std::cout << "The smalles pcl size is equal " << smallestCloud->width * smallestCloud->height <<std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::PointXYZRGB centroid;
    pcl::computeCentroid(*smallestCloud, centroid);

    for (pcl::PointXYZRGB& point : smallestCloud->points)
    {
        point.x -= centroid.x;
        point.y -= centroid.y;
        point.z -= centroid.z;
    }
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.0001);

    seg.setInputCloud (smallestCloud);
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
    viewer.addPointCloud<pcl::PointXYZRGB>(smallestCloud, "cloud");
    viewer.addPointCloud(smallestCloud, "point_cloud");
    
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
