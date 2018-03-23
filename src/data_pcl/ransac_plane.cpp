// ROS

#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>

// PCL

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>


typedef pcl::PointXYZ PointT;

using namespace std;

class testPCL{
public:
    testPCL(ros::NodeHandle nh): _nh(nh){
        initialize();
    }

    void initialize(){

        // Get node name
        name = ros::this_node::getName();
        // Sunscriber
        sub = _nh.subscribe("/velodyne_points", 1000, &testPCL::cloud_cb, this);
        // Publisher
        pub_plane = _nh.advertise<sensor_msgs::PointCloud2 > ("filtered_plane", 1000);
        pub_scene = _nh.advertise<sensor_msgs::PointCloud2 > ("filtered_cloud", 1000);
    }

    void spin(){

        ros::spin();
    }

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){

        pcl::PointCloud<PointT>::Ptr input_cloud_ (new pcl::PointCloud<PointT>);
        cout << "input cloud size " << input->width << " " << input->height << endl;

        pcl::fromROSMsg (*input, *input_cloud_);
        cout << "cloud size " << input_cloud_->width << " " << input_cloud_->height << endl;
        cout << "point " << input_cloud_->points[10].x << " " << input_cloud_->points[10].y <<
                " " << input_cloud_->points[10].z << endl;

        pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT>);

        // Narrow down horizontal field of view
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (input_cloud_);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (0.4, 100.0); // from lidar specification measurement range up to 100 m
        //pass.setFilterLimitsNegative (true);
        pass.filter (*input_cloud);

        cout << "new cloud size " << input_cloud->width << " " << input_cloud->height << endl;

        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

        double max_distance = 0.04; // according to LiDAR specification accuracy 3 cm

        // find a plane
        pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients); //container for the model coefficients
        pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices); //container for inlier idx
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        // Segmentation
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold(max_distance);
        seg.setInputCloud (input_cloud);
        seg.segment (*plane_inliers, *plane_coefficients);

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (input_cloud);
        extract.setIndices (plane_inliers);
        extract.setNegative (false);

        // extract planar inliers
        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
        extract.filter (*cloud_plane);

        // Remove the planar inliers, extract the rest
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

    // Subscriber
    ros::Subscriber sub;

    // Algorithm parameters
    double max_distance;

};

int main(int argc, char** argv)
{
    // ROS init
    ros::init (argc,argv,"ransac_plane");
    ros::NodeHandle nh("~");
    testPCL t_pcl(nh);

    t_pcl.spin();

    return 0;
}
