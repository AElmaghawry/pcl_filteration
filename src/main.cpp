#include <bladAngleDetection.hpp>

class CSVWriter
{
private:
    std::string filename;
    std::ofstream file;

public:
    CSVWriter(const std::string &filename) : filename(filename)
    {
        file.open(filename);
        if (!file.is_open())
        {
            std::cout << "Failed to open file: " << filename << std::endl;
        }
    }

    ~CSVWriter()
    {
        if (file.is_open())
        {
            file.close();
        }
    }

    void writeRow(const std::vector<std::string> &row)
    {
        if (!file.is_open())
        {
            std::cout << "File is not open." << std::endl;
            return;
        }

        for (size_t i = 0; i < row.size(); ++i)
        {
            file << row[i];
            if (i != row.size() - 1)
                file << ",";
        }
        file << "\n";
    }
};

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
        std::cout << "\033[37m"
                  << "Loaded "
                  << "\033[93m" << cloud->width * cloud->height
                  //   << " :"
                  //   << file_name
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
        // pcl::visualization::PCLVisualizer viewer("Cluster viewer");
        // viewer.addPointCloud(colored_cloud, "cluster viewer");
        // viewer.setBackgroundColor(0.0, 0.0, 0.0);
        // viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(colored_cloud, normals);
        // viewer.spinOnce();
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
            std::cout << "\033[37m"
                      << "Cluster " << i + 1
                      << " has "
                      << "\033[94m" << clusters[i].indices.size()
                      << "\033[37m"
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
    std::vector<double> bladeCoeff{};
    std::vector<double> centriodBlades{};

    std::string filename = "../data/measuredAngles.csv";
    CSVWriter writer(filename);
    writer.writeRow({"Blade No.", "Measured Angles"});

    std::string direcoty_name = "../data/data_set/";
    // std::string file_name = "../data/data_set/4/4.pcd";

    std::cout << "\033[93m"
              << "Enter the number of the data set : ";
    int userSelection{0};
    std::cin >> userSelection;

    for (int counter{1}; counter <= userSelection; ++counter)
    {
        std::string file_name = direcoty_name + std::to_string(counter) + "/" + std::to_string(counter) + ".pcd";
        std::cout << "\033[37m"
                  << "The Data set Directory : " << file_name << std::endl;
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

        // std::cout << clusters.size() << std::endl;

        std::string cloud_of_intreset_dir = "../data/cloud_cluster_"; /// home/jeo/ku/pcl_filteration/data/cloud_cluster_0.pcd

        std::vector<std::string> files_paths{};
        PointCloudProcessing pcSelection;

        for (int i{0}; i < clusters.size(); i++)
        {
            files_paths.push_back(cloud_of_intreset_dir + std::to_string(i) + ".pcd");
        }
        // std::cout << files_paths[1] << std::endl;

        std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointClouds;
        int i{0};
        for (const std::string &filepath : files_paths)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile<pcl::PointXYZRGB>(filepath, *cloud);
            pointClouds.push_back(cloud);
            i++;
            // std::cout << "Loading the pcd files ......" << i << std::endl;
        }

        size_t smallestSize = std::numeric_limits<size_t>::max();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr smallestCloud;

        for (const auto &cloud : clusters_vector)
        {
            size_t cloudSize = cloud->size();
            if (cloudSize < smallestSize)
            {
                smallestSize = cloudSize;
                smallestCloud = cloud;
            }
        }

        std::cout << "\033[37m"
                  << "The smalles pcl size is equal = "
                  << "\033[93m" << smallestCloud->width * smallestCloud->height << std::endl;

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::PointXYZRGB centroid;
        pcl::computeCentroid(*smallestCloud, centroid);

        for (pcl::PointXYZRGB &point : smallestCloud->points)
        {
            point.x -= centroid.x;
            point.y -= centroid.y;
            point.z -= centroid.z;
        }
        std::cout << "The Centriod of blade No." << counter << ": x = "
                  << centroid.x << ", y = "
                  << centroid.y << ", z ="
                  << centroid.z << std::endl;

        centriodBlades.push_back(centroid.x);
        centriodBlades.push_back(centroid.y);
        centriodBlades.push_back(centroid.z);

        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.0001);

        seg.setInputCloud(smallestCloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            PCL_ERROR("Could not estimate a planar model for the given dataset.");
            return (-1);
        }

        std::cerr << "\033[37m"
                  << "Model coefficients: "
                  << "\033[96m"
                  << coefficients->values[0] << " "
                  << coefficients->values[1] << " "
                  << coefficients->values[2] << " "
                  << coefficients->values[3] << std::endl;

        bladeCoeff.push_back(coefficients->values[0]);
        bladeCoeff.push_back(coefficients->values[1]);
        bladeCoeff.push_back(coefficients->values[2]);
        // bladeCoeff.push_back(coefficients->values[3]);

        pcl::ModelCoefficients::Ptr coefficientsRef(new pcl::ModelCoefficients);
        coefficientsRef->values.push_back(0.0);
        coefficientsRef->values.push_back(0.0);
        coefficientsRef->values.push_back(1.0);
        coefficientsRef->values.push_back(0.0);

        // pcl::visualization::PCLVisualizer viewer("Point Cloud Visualization");

        // viewer.setBackgroundColor(0.0, 0.0, 0.0);
        // viewer.addPointCloud<pcl::PointXYZRGB>(smallestCloud, "cloud");
        // viewer.addPointCloud(smallestCloud, "point_cloud");

        // viewer.addPlane(*coefficients, "plane");
        // viewer.addPlane(*coefficientsRef, "refrence");

        // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "plane");
        // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "refrence");

        std::cout << "\033[32m"
                  << "________________________________________" << std::endl;

        std::cout << "Measured Angle between refrence frame and Surface of the clustred blade is = "
                  << "\033[96m" << calcAngelTwoVectors(coefficientsRef, coefficients)
                  << "\033[32m"
                  << " degrees"
                  << std::endl;

        /*
            .csv filling template
            {"Blade No.", "Measured Angles"}
        */

        writer.writeRow({std::to_string(counter), std::to_string(calcAngelTwoVectors(coefficientsRef, coefficients))});

        // viewer.addCoordinateSystem(0.1);
        // viewer.spin();
    }
    // std::cout << "\033[33m"
    //           << "-------------------------------------------------------------------" << std::endl;
    // std::cout << "The size of the of the centriod blade is equal = " << centriodBlades.size() << std::endl;
    // std::cout << "The Coff. for the recorded blades are equal = " << bladeCoeff.size() << std::endl;

    // Eigen::Vector3d v(0.00743252, -0.781734, 0.860328);
    Eigen::Vector3d normalizedVector(bladeCoeff[0], bladeCoeff[1], bladeCoeff[2]);
    normalizedVector = normalizedVector.normalized();

    std::vector<double> delta{};

    for (int itr{3}; itr < (centriodBlades.size()); itr += 3)
    {
        delta.push_back(centriodBlades[itr] - centriodBlades[0]);
        delta.push_back(centriodBlades[itr + 1] - centriodBlades[1]);
        delta.push_back(centriodBlades[itr + 2] - centriodBlades[2]);
    }

    int pointCounter{0};
    std::string offsetMeasurmentsFile = "../data/offsetCalc.csv";
    CSVWriter writerOffsetCalc(offsetMeasurmentsFile);

    writerOffsetCalc.writeRow({"Offset Calculation"});
    for (int itr{0}; itr < (delta.size()); itr += 3)
    {
        Eigen::Vector3d deltaVector(delta[itr], delta[itr + 1], delta[itr + 2]);
        /*
        un comment the next line of code for to print the measured offest values on the termianl
        */
        // std::cout << "Dot product of between refrence blade and blade No.: "
        //           << pointCounter << " is equal = "
        //           << deltaVector.dot(normalizedVector) << std::endl;
        pointCounter++;
        writerOffsetCalc.writeRow({std::to_string(deltaVector.dot(normalizedVector))});
    }

    pointCounter = 0;
    std::string rotationAnglesCsv = "../data/rotAngles.csv";
    CSVWriter rotationAngles(rotationAnglesCsv);
    rotationAngles.writeRow({"Roll","Pitch","Yaw"});

    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};

    for (int i{0}; i < (bladeCoeff.size()); i += 3)
    {

        Eigen::Vector3d vFrame(bladeCoeff[i], bladeCoeff[i + 1], bladeCoeff[i + 2]);
        Eigen::Matrix3d rotMatrix;
        rotMatrix.col(2) = vFrame.normalized();
        rotMatrix.col(0) = Eigen::Vector3d::UnitX().cross(rotMatrix.col(2));
        rotMatrix.col(1) = rotMatrix.col(2).cross(rotMatrix.col(0));
        Eigen::Vector3d euler_angles = rotMatrix.eulerAngles(2, 0, 2);
        std::cout << BOLDRED << "The Rotation Matrix is of blade No. " << pointCounter << RESET << std::endl
                  << rotMatrix << std::endl;
        std::cout << WHITE << "---------------------------------------" << std::endl;
        // std::cout << "The Roll,Pitch,Yaw Angles blade No. ->" << pointCounter << std::endl
        //           << euler_angles.transpose() << std::endl;

        std::vector<double> stdVector(euler_angles.data(), euler_angles.data() + euler_angles.size());

        roll = stdVector[0];
        pitch = stdVector[1];
        yaw = stdVector[2];

        /*
        This is to print Roll,Pitch, and Yaw Angles
        */
        std::cout << YELLOW << "The Measured Angles " << std::endl;
        std::cout << WHITE << "Roll Angle = " << CYAN << roll << std::endl;
        std::cout << WHITE << "Pitch Angle = " << CYAN << pitch << std::endl;
        std::cout << WHITE << "Yaw Angle = " << CYAN << yaw << std::endl;
        std::cout << WHITE << "------------------------------------" << std::endl;

        rotationAngles.writeRow({std::to_string(roll), std::to_string(pitch), std::to_string(yaw)});

        pointCounter++;
    }

    // std::cout << "The Normalized vector " << normalizedVector << endl ;

    // std::cout << bladeCoeff[0]<<bladeCoeff[1]<<bladeCoeff[2] << std::endl ;

    //  v.dot(w)

    return 0;
}
