#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include "fstream"  
#include <dirent.h>
#include <stdio.h>
#include <string>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include "common.h"
#include "pcl_tools.hpp"
using namespace std;

using namespace PCL_TOOLS;
int main()
{
    pcl::PointCloud<PointType> cloud_1;
    pcl::PointCloud<PointType> cloud_2;
    pcl::PointCloud<PointType> cloud_3;
    pcl::PointCloud<PointType> cloud_4;
    pcl::PCDReader reader;
    string surfPath = "/home/beihai/catkin_ws/src/loam_livox/pcd/surface/";
    string cornerPath = "/home/beihai/catkin_ws/src/loam_livox/pcd/corner/";
//    Eigen::Quaterniond rotation=Eigen::Quaterniond(0.999889,0.002404,0.0067009,0.013092);
//    Eigen::Vector3d trans=Eigen::Vector3d(79.2074,1.11397,-0.85304);
//    reader.read(rootdirPath,cloud_1);
//    for(int i=0;i<cloud_1.points.size();i++)
//    {
//	Eigen::Vector3d point_curr(cloud_1[i].x,cloud_1[i].y,cloud_1[i].z);
//	Eigen::Vector3d point_w;
//        point_w=rotation.inverse()*point_curr-rotation.inverse()*trans;
//	cloud_1[i].x=point_w.x();
//	cloud_1[i].y=point_w.y();
//	cloud_1[i].z=point_w.z();
//	cloud_1[i].intensity=cloud_1[i].intensity;
//    }
    for(int i=5000;i<=5100;i++)
    {
        reader.read(surfPath+to_string(i)+".pcd",cloud_1);
        cloud_2=cloud_2+cloud_1;
        //cerr<<i;
    }
    pcl::PCDWriter writer;
    writer.write("surface5000_5100.pcd",cloud_2);

    for(int i=5000;i<=5100;i++)
    {
        reader.read(cornerPath+to_string(i)+".pcd",cloud_3);
        cloud_4=cloud_4+cloud_3;
        //cerr<<i;
    }
    writer.write("corner5000_5100.pcd",cloud_4);
    return 0;
}
