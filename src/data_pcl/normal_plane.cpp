// ROS
#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


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

        pcl::PointCloud<PointT>::Ptr input_cloud_ (new pcl::PointCloud<PointT>);

        pcl::fromROSMsg (*input, *input_cloud_);

        // Narrow down horizontal field of view

        pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT>);
        pcl::PassThrough<pcl::PointXYZ> pass;

        pass.setInputCloud (input_cloud_);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (0.4, 100.0); // from lidar specification measurement range up to 100 m
        //pass.setFilterLimitsNegative (true);
        pass.filter (*input_cloud);
        cout << "cloud size after pass through " << input_cloud->width << " " << input_cloud->height << endl;

        // Estimate point normals

        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<PointT, pcl::Normal> ne;

        ne.setSearchMethod (tree);
        ne.setInputCloud (input_cloud);
        ne.setKSearch (30); // neighbouring area for search
        ne.compute (*cloud_normals);

        cout << "cloud normals " << cloud_normals->width << " " << cloud_normals->height << endl;
        cout << "cloud normals values " << cloud_normals->points[10].normal_x << " "
             << cloud_normals->points[10].normal_y << " " << cloud_normals->points[10].normal_z << endl;

        // Segmentation

        double max_distance = 0.05; // according to LiDAR specification accuracy 3 cm

        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight (0.01);
        seg.setMethodType (pcl::SAC_RANSAC);
        //seg.setMaxIterations (100);
        seg.setDistanceThreshold (max_distance);
        seg.setInputCloud (input_cloud);
        seg.setInputNormals (cloud_normals);
        // Obtain the plane inliers and coefficients
        seg.segment (*inliers_plane, *coefficients_plane);

        // Extract the planar inliers from the input cloud

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (input_cloud);
        extract.setIndices (inliers_plane);
        extract.setNegative (false);

        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
        extract.filter (*cloud_plane);

        // Remove the planar inliers, extract the rest

        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

        extract.setNegative (true);
        extract.filter (*cloud_filtered);
        extract_normals.setNegative (true);
        extract_normals.setInputCloud (cloud_normals);
        extract_normals.setIndices (inliers_plane);
        extract_normals.filter (*cloud_normals2);
        //input_cloud.swap (cloud_normals2);

        // Publish points

        sensor_msgs::PointCloud2 cloud_publish;
        pcl::toROSMsg(*cloud_filtered, cloud_publish);
        cloud_publish.header = input->header;
        pub_scene.publish(cloud_publish);

        sensor_msgs::PointCloud2 plane;
        pcl::toROSMsg(*cloud_plane, plane);
        plane.header = input->header;
        pub_plane.publish(plane);

        sensor_msgs::PointCloud2 normals;
        pcl::toROSMsg(*cloud_normals, normals);
        normals.header = input->header;
        pub_normals.publish(normals);

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
    ros::init (argc,argv,"normal_plane");
    ros::NodeHandle nh("~");
    filterPCL filter_pcl(nh);

    filter_pcl.spin();

    return 0;
}
