#include <iostream>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <iterator>

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <utility>

using namespace std;
ros::Publisher cc_pos;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)

{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      /* Creating the KdTree from input point cloud*/
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::fromROSMsg (*input, *input_cloud);

  tree->setInputCloud (input_cloud);

  /* Here we are creating a vector of PointIndices, which contains the actual index
   * information in a vector<int>. The indices of each detected cluster are saved here.
   * Cluster_indices is a vector containing one instance of PointIndices for each detected
   * cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
   */
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.3);
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (600);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
cout<<"PCL init successfull\n";
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract (cluster_indices);
cout<<"PCL extract successfull\n";
  /* To separate each cluster out of the vector<PointIndices> we have to
   * iterate through cluster_indices, create a new PointCloud for each
   * entry and write all points of the current cluster in the PointCloud.
   */
  //pcl::PointXYZ origin (0,0,0);
  //float mindist_this_cluster = 1000;
  //float dist_this_point = 1000;

  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  // Vector of cluster pointclouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_vec;

  // Cluster centroids
  std::vector<pcl::PointXYZ> clusterCentroids;

  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
   {
        float x=0.0; float y=0.0;
         int numPts=0;
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          for(pit = it->indices.begin(); pit != it->indices.end(); pit++)
          {

                  cloud_cluster->points.push_back(input_cloud->points[*pit]);

                  x+=input_cloud->points[*pit].x;
                  y+=input_cloud->points[*pit].y;
                  numPts++;

                  //dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
                  //                                          origin);
                  //mindist_this_cluster = std::min(dist_this_point, mindist_this_cluster);
          }

      pcl::PointXYZ centroid;
      centroid.x=x/numPts;
      centroid.y=y/numPts;
      centroid.z=0.0;

      cluster_vec.push_back(cloud_cluster);

      //Get the centroid of the cluster
      clusterCentroids.push_back(centroid);

    }
    cout << clusterCentroids.size() << endl;
   // cout<<"cluster_vec got some clusters\n";

    //Ensure at least 6 clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < 6){
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0,0,0));
      cluster_vec.push_back(empty_cluster);
    }

     while (clusterCentroids.size()<6)
    {
      pcl::PointXYZ centroid;
      centroid.x=0.0;
      centroid.y=0.0;
      centroid.z=0.0;

       clusterCentroids.push_back(centroid);
    }

    std_msgs::Float32MultiArray cc;
    for(int i=0;i<6;i++)
    {
        cc.data.push_back(clusterCentroids.at(i).x);
        cc.data.push_back(clusterCentroids.at(i).y);
        cc.data.push_back(clusterCentroids.at(i).z);

    }
    // cout<<"6 clusters initialized\n";

    cc_pos.publish(cc);// Publish cluster mid-points.

}


int main(int argc, char** argv)
{
    // ROS init
    ros::init (argc,argv,"pc_kdtree");
    ros::NodeHandle nh;

    cout<<"About to setup callback\n";

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1000, cloud_cb);
    cc_pos = nh.advertise<std_msgs::Float32MultiArray> ("centroids", 1000);
    ros::spin();


}
