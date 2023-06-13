#include <bladAngleDetection.hpp>
#include <list>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstring>

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

    void saveClusterPointClouds(const std::vector<pcl::PointIndices> &clusters, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud, int minClusterSize)
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
    std::string file_name = "../data/0.pcd";

    PointCloudProcessing pcProcessing;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!pcProcessing.loadPointCloud(file_name, cloud))
        return -1;

    RegionGrowingSegmentation regionSegmentation;
    regionSegmentation.performSegmentation(cloud);

    const std::vector<pcl::PointIndices> &clusters = regionSegmentation.getClusters();
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud = regionSegmentation.getColoredCloud();

    pcProcessing.saveClusterPointClouds(clusters, colored_cloud, 1500);

    pcProcessing.visualizePointCloud(colored_cloud, regionSegmentation.getNormals());

    std::cout << clusters.size() << std::endl;

    std::string cloud_of_intreset_dir = "../data/cloud_cluster_"; ///home/jeo/ku/pcl_filteration/data/cloud_cluster_0.pcd

    std::vector<std::string> files_paths{};
    PointCloudProcessing pcSelection;
    
    for (int i{0}; i < clusters.size(); i++)
    {
        files_paths.push_back(cloud_of_intreset_dir + std::to_string(i) + ".pcd");
    }
    // std::cout << files_paths[1] << std::endl;

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

    for (const auto& cloud : pointClouds)
    {
        size_t cloudSize = cloud->size();
        if (cloudSize < smallestSize)
        {
            smallestSize = cloudSize;
            smallestCloud = cloud;
        }
    }

    std::cout << "The smalles pcl size is equal " << smallestCloud->width * smallestCloud->height <<std::endl;

    return 0;
}
