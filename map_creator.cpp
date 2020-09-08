#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include<string>
#include<math.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
using namespace std;
int i,j;
pcl::PCDWriter writer;
pcl::PointCloud<pcl::PointXYZ> corner_input,surface_input;
pcl::PointCloud<pcl::PointXYZ> corner_sum,surface_sum;
void corner_Callback (const sensor_msgs::PointCloud2 input)
{
    pcl::fromROSMsg (input,corner_input);
    corner_sum=corner_sum+corner_input;
    std::cerr<<"corner_number: "<<corner_sum.points.size()<<std::endl;
    //writer.write("corner_map.pcd",corner_sum);
    writer.write("/home/beihai/my_workspace/src/livox_aftermaped/lidar_frames/corners/"+std::to_string(i)+".pcd",corner_input);
    i++;
}
void surface_Callback (const sensor_msgs::PointCloud2 input)
{
    pcl::fromROSMsg (input,surface_input);
    surface_sum=surface_sum+surface_input;
    std::cerr<<"surface_number: "<<surface_sum.points.size()<<std::endl;
    //writer.write("surface_map.pcd",surface_sum);
    writer.write("/home/beihai/my_workspace/src/livox_aftermaped/lidar_frames/surface/"+std::to_string(j)+".pcd",surface_input);
    j++;
}
int main (int argc, char** argv)
{
// Initialize ROS
    ros::init (argc, argv, "map_creator");
    ros::NodeHandle nh;

// Create a ROS subscriber for the input point cloud
    ros::Subscriber corner_sub = nh.subscribe ("pc2_corners", 5, corner_Callback);
    ros::Subscriber surface_sub = nh.subscribe ("pc2_surface", 5, surface_Callback);

// Create a ROS publisher for the output point cloud
    //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    //map_pub= nh.advertise<nav_msgs::OccupancyGrid> ("map", 2,true);
// Spin
    ros::MultiThreadedSpinner spinner(2); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
    //ros::spin ();
}




