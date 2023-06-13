#include<bladAngleDetection.hpp>

int main(int argc, char **argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string file_name = "/home/joe/pcl_filteration/data/0.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout   << "Loaded "
                << cloud->width * cloud->height
                << " :"
                << file_name
                << std::endl;

    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    // pcl::IndicesPtr indices(new std::vector<int>);
    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.0, 1.0);
    // pass.filter(*indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(1000);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud);
    // reg.setIndices (indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);
    pcl::PCDWriter writer;
    std::vector<pcl::PointIndices> clusters;
    // std::vector <pcl::PointIndices> exctracted_classes;
    reg.extract(clusters);
    std::vector<pcl::PointIndices> blade_cluster{};
    for (int i{0}; i < clusters.size(); ++i)
    {
        if (clusters[i].indices.size() > 1500)
        {
            blade_cluster.push_back(clusters[i]);
            std::cout   << "Cluster " << i + 1
                        << " has "
                        << clusters[i].indices.size()
                        << " points." << std::endl;
        }
    }
    std::cout << "-------------------------"<<endl ; 

    // std::cout << "The size of the the cluster is equal to = " << blade_cluster.size() << endl;
    // std::cout << "These are the indices of the points of the initial" <<
    // std::endl << "cloud that belong to the first cluster:" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::PCLVisualizer viewer("Cluster viewer");
    // viewer.setSize(630,480); 
    viewer.addPointCloud(colored_cloud, "cluster viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    int j = 0;

    for (const auto & indices : blade_cluster)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto & pt : indices.indices)
        {
            cloud_cluster->push_back((*colored_cloud)[pt]); //*
            // std::cout<<"here"<<endl ;
        }
        // std::cout<<"here1"<<endl ;
        cloud_cluster->width = indices.indices.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // cloud_cluster->resize(cloud_cluster->width * cloud_cluster->height);
        // cloud_cluster->header = colored_cloud->header ;  

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        std::stringstream ss;
        ss << "../cloud_cluster_" << j << ".pcd";
        // std::cout<<"here3"<<endl ;
        // pcl::io::savePCDFileASCII(ss.str(),*cloud_cluster);
        std::cout << cloud_cluster->width << std::endl;
        std::cout << cloud_cluster->height << std::endl;

        writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_cluster, false);
        j++;
        // cloud_cluster->clear () ; 
    }

    viewer.spin();
    return (0);
}