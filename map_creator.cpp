#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Odometry.h"
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include<string>
#include<math.h>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include "common.h"
#include "tools_logger.hpp"
#include "tools_random.hpp"
#include<time.h>
#include "pcl_tools.hpp"
#include "ceres_icp.hpp"

// PCL specific includes
//#include <pcl/ros/conversions.h>
#include<pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
using namespace std;
using namespace Common_tools;
using namespace PCL_TOOLS;
int i,j;
pcl::PCDWriter writer;
pcl::PointCloud<PointType> corner_input,surface_input;
pcl::PointCloud<PointType> corner_sum,surface_sum;
clock_t c_start,c_end, s_start,s_end;

void corner_Callback (const sensor_msgs::PointCloud2 input)
{
    c_start = clock();
    pcl::fromROSMsg (input,corner_input);
    corner_sum=corner_sum+corner_input;
    std::cerr<<"corner_number: "<<corner_sum.points.size()<<std::endl;
    writer.write("corner_map.pcd",corner_sum);
    //writer.write("/home/beihai/my_workspace/src/livox_aftermaped/lidar_frames/corners/"+std::to_string(i)+".pcd",corner_input);
    i++;
    c_end = clock();
    cerr<<(c_end-c_start)/CLOCKS_PER_SEC<<endl;
}
void surface_Callback (const sensor_msgs::PointCloud2 input)
{
    s_start = clock();

    pcl::fromROSMsg (input,surface_input);
    surface_sum=surface_sum+surface_input;
    std::cerr<<"surface_number: "<<surface_sum.points.size()<<std::endl;
    writer.write("surface_map.pcd",surface_sum);
    //writer.write("/home/beihai/my_workspace/src/livox_aftermaped/lidar_frames/surface/"+std::to_string(j)+".pcd",surface_input);
    j++;
    s_end = clock();
    cerr<<(s_end-s_start)/CLOCKS_PER_SEC<<endl;
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




