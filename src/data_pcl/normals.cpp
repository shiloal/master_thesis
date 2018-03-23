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
#include <pcl/filters/extract_indices.h>

#include <math.h> // sqrt

typedef pcl::PointXYZ PointT;

using namespace std;
const double PI = 3.141592653589793;


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
        ne.setKSearch (50); // neighbouring area for search
        ne.compute (*cloud_normals);

        // normals are normalised == vector length = 1
        /*double normal_length1 = sqrt(pow(cloud_normals->points[10].normal_x,2) + pow(cloud_normals->points[10].normal_y,2) + pow(cloud_normals->points[10].normal_z,2));
        double normal_length2 = sqrt(pow(cloud_normals->points[100].normal_x,2) + pow(cloud_normals->points[100].normal_y,2) + pow(cloud_normals->points[100].normal_z,2));
        double normal_length3 = sqrt(pow(cloud_normals->points[1000].normal_x,2) + pow(cloud_normals->points[1000].normal_y,2) + pow(cloud_normals->points[1000].normal_z,2));

        cout << "normal_length " << normal_length1 << " "
             << normal_length2 << " " << normal_length3 << endl;

        */

        cout << "cloud normals " << cloud_normals->width << " " << cloud_normals->height << endl;
        cout << "cloud normals values " << cloud_normals->points[10].normal_x << " "
             << cloud_normals->points[10].normal_y << " " << cloud_normals->points[10].normal_z << endl;

        // Merge point cloud with its mornals

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields (*input_cloud, *cloud_normals, *cloud_with_normals);

        pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices); //container for inlier idx

        for (int i = 0; i < cloud_with_normals->points.size(); ++i){
            // normal angle from vertical
            double slope_angle = acos (cloud_with_normals->points[i].normal_z) * 180.0 / PI;
            if (slope_angle < 20){
                cout << "slope angle " << slope_angle << endl;
                plane_inliers->indices.push_back(i);
            }
        }

        // Extract the planar inliers from the input cloud
        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        extract.setInputCloud(input_cloud);
        extract.setIndices(plane_inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane);

        // Remove the planar inliers, extract the rest
        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());

        extract.setNegative (true);
        extract.filter (*cloud_filtered);
        input_cloud.swap (cloud_filtered);

        // Publish points

        sensor_msgs::PointCloud2 cloud_publish;
        pcl::toROSMsg(*input_cloud, cloud_publish);
        cloud_publish.header = input->header;
        pub_scene.publish(cloud_publish);

        sensor_msgs::PointCloud2 plane;
        pcl::toROSMsg(*cloud_plane, plane);
        plane.header = input->header;
        pub_plane.publish(plane);

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
    ros::init (argc,argv,"normals");
    ros::NodeHandle nh("~");
    filterPCL filter_pcl(nh);

    filter_pcl.spin();

    return 0;
}
