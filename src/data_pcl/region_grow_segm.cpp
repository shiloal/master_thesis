// ROS
#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>

// PCL
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>


typedef pcl::PointXYZ PointT;

using namespace std;

class filterPCL{
public:
    filterPCL(ros::NodeHandle nh): _nh(nh){
        initialize();
    }

    void initialize(){

        // Get node name
        name = ros::this_node::getName();
        // Sunscriber
        sub = _nh.subscribe("/velodyne_points", 1000, &filterPCL::cloud_cb, this);
        // Publisher
        pub_plane = _nh.advertise<sensor_msgs::PointCloud2 > ("filtered_plane", 1000);
        pub_scene = _nh.advertise<sensor_msgs::PointCloud2 > ("filtered_cloud", 1000);
        pub_normals = _nh.advertise<sensor_msgs::PointCloud2 > ("normals_cloud", 1000);
    }

    void spin(){

        ros::spin();
    }

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){

        pcl::PointCloud<PointT>::Ptr cloud_ (new pcl::PointCloud<PointT>);

        pcl::fromROSMsg (*input, *cloud_);

        // Narrow down horizontal field of view
        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        pcl::PassThrough<pcl::PointXYZ> pass;

        pass.setInputCloud (cloud_);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (0.4, 100.0); // from lidar specification measurement range up to 100 m
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud);

        pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (cloud);
        normal_estimator.setKSearch (30);
        normal_estimator.compute (*normals);
/*
        pcl::IndicesPtr indices (new std::vector <int>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (0.4, 100.0);
        pass.filter (*indices);
*/
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize (100);
        reg.setMaxClusterSize (1000000);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (50);
        reg.setInputCloud (cloud);
        //reg.setIndices (indices);
        reg.setInputNormals (normals);
        reg.setSmoothnessThreshold (1.5 / 180.0 * M_PI);
        reg.setCurvatureThreshold (1.0);

        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);
        std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
        std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
        std::cout << "These are the indices of the points of the initial" <<
        std::endl << "cloud that belong to the first cluster:" << std::endl;
        int counter = 0;
        while (counter < clusters[0].indices.size ()){

            std::cout << clusters[0].indices[counter] << ", ";
            counter++;
            if (counter % 10 == 0)
                std::cout << std::endl;
        }
        std::cout << std::endl;

        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

        // Publish points

        sensor_msgs::PointCloud2 cloud_publish;
        pcl::toROSMsg(*colored_cloud, cloud_publish);
        cloud_publish.header = input->header;
        pub_scene.publish(cloud_publish);

        //pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
        pcl::visualization::CloudViewer viewer ("Cluster viewer");
        viewer.showCloud(colored_cloud);
        while (!viewer.wasStopped ()){
        }


/*
        sensor_msgs::PointCloud2 plane;
        pcl::toROSMsg(*cloud_plane, plane);
        plane.header = input->header;
        pub_plane.publish(plane);

        sensor_msgs::PointCloud2 normals;
        pcl::toROSMsg(*cloud_normals, normals);
        normals.header = input->header;
        pub_normals.publish(normals);
*/
    }

private:

    // Node
    ros::NodeHandle _nh;
    std::string name;

    // Publishers
    ros::Publisher pub_plane; // Display inliers for a plane
    ros::Publisher pub_scene;
    ros::Publisher pub_normals;

    // Subscriber
    ros::Subscriber sub;

    // Algorithm parameters
    double max_distance;

};

int main(int argc, char** argv)
{
    // ROS init
    ros::init (argc,argv,"region_grow_segm");
    ros::NodeHandle nh("~");
    filterPCL filter_pcl(nh);

    filter_pcl.spin();

    return 0;
}
