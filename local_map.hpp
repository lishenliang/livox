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
#include<ctime>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include "common.h"
#include "tools_logger.hpp"
#include "tools_random.hpp"

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
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/impl/io.hpp>

class Local_map
{
    public:
    int numbers;
    float radius=0.3;
    pcl::PCDWriter writer;
    Eigen::Matrix4d corner_transformation_matrix = Eigen::Matrix4d::Identity ();
    Eigen::Matrix4d surface_transformation_matrix = Eigen::Matrix4d::Identity ();
    //pcl::KdTreeFLANN<PointType>::Ptr corner_curr_kdtree;
    //pcl::KdTreeFLANN<PointType>::Ptr surface_curr_kdtree;

    void
    print4x4Matrix (const Eigen::Matrix4d & matrix)
    {
      printf ("Rotation matrix :\n");
      printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
      printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
      printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
      printf ("Translation vector :\n");
      printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    }

    int local_map_creater(pcl::PointCloud< PointType >::Ptr in_laser_cloud_corner_from_map,
                          pcl::PointCloud< PointType >::Ptr in_laser_cloud_surf_from_map,
                          pcl::search::KdTree<PointType>::Ptr   kdtree_corner_from_map,
                          pcl::search::KdTree<PointType>::Ptr   kdtree_surf_from_map,
                          pcl::PointCloud< PointType >::Ptr laserCloudCornerStack,
                          pcl::PointCloud< PointType >::Ptr laserCloudSurfStack )
    {
        pcl::search::KdTree<PointType>::Ptr corner_curr_kdtree( new pcl::search::KdTree<PointType>);
        pcl::search::KdTree<PointType>::Ptr surface_curr_kdtree( new pcl::search::KdTree<PointType>);
//        pcl::search::KdTree<PointType>::Ptr corner_kdtree( new pcl::search::KdTree<PointType>);
//        pcl::search::KdTree<PointType>::Ptr surface_kdtree( new pcl::search::KdTree<PointType>);
//        corner_kdtree = &kdtree_corner_from_map;
//        surface_kdtree = &kdtree_surf_from_map;
//        pcl::PointCloud< PointType > corner_localmap;
        pcl::PointCloud<PointType>::Ptr corner_localmap (new pcl::PointCloud<PointType>);

        pcl::PointCloud< pcl::PointXYZ > corner_localmapXYZ;
        pcl::PointCloud< pcl::PointXYZ > laserCloudCornerStackXYZ;

        pcl::PointCloud< PointType > surf_localmap;
        PointType searchPoint;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        std::cerr<<laserCloudCornerStack->points.size()<<std::endl;
        for(int i=0;i<laserCloudCornerStack->points.size();i++)
        {
            searchPoint=laserCloudCornerStack->points[i];
            if ( kdtree_corner_from_map->radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            {
    //            for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    //              std::cout << "    "  <<in_laser_cloud_corner_from_map->points[ pointIdxRadiusSearch[ i ] ].x
    //                        << " " << in_laser_cloud_corner_from_map->points[ pointIdxRadiusSearch[ i ] ].y
    //                        << " " << in_laser_cloud_corner_from_map->points[ pointIdxRadiusSearch[ i ] ].z
    //                        << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
                //numbers+=pointIdxRadiusSearch.size();
                //std::cout<<pointIdxRadiusSearch.size()<<std::endl;
                for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                {
                    corner_localmap->points.push_back(in_laser_cloud_corner_from_map->points[i]);
                }
            }
        }
        std::cerr<<corner_localmap->points.size()<<std::endl;

        //Transform
        Eigen::Quaterniond rotation=Eigen::Quaterniond(1,0,0,0);
        Eigen::Vector3d trans=Eigen::Vector3d(0,0,0);
        for(int i=0;i<laserCloudCornerStack->points.size();i++)
        {
            Eigen::Vector3d point_curr(laserCloudCornerStack->points[i].x,laserCloudCornerStack->points[i].y,laserCloudCornerStack->points[i].z);
            Eigen::Vector3d point_w;
            point_w=rotation.inverse()*point_curr-rotation.inverse()*trans;
            laserCloudCornerStack->points[i].x=point_w.x();
            laserCloudCornerStack->points[i].y=point_w.y();
            laserCloudCornerStack->points[i].z=point_w.z();
            laserCloudCornerStack->points[i].intensity=laserCloudCornerStack->points[i].intensity;
        }
        for(int i=0;i<laserCloudSurfStack->points.size();i++)
        {
            Eigen::Vector3d point_curr(laserCloudSurfStack->points[i].x,laserCloudSurfStack->points[i].y,laserCloudSurfStack->points[i].z);
            Eigen::Vector3d point_w;
            point_w=rotation.inverse()*point_curr-rotation.inverse()*trans;
            laserCloudSurfStack->points[i].x=point_w.x();
            laserCloudSurfStack->points[i].y=point_w.y();
            laserCloudSurfStack->points[i].z=point_w.z();
            laserCloudSurfStack->points[i].intensity=laserCloudSurfStack->points[i].intensity;
        }

//        corner_curr_kdtree->setInputCloud (laserCloudCornerStack);
//        surface_curr_kdtree->setInputCloud (laserCloudSurfStack);
        //ICP
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> corner_icp;
//        corner_icp.setSearchMethodSource(corner_curr_kdtree);
//        corner_icp.setSearchMethodTarget(kdtree_corner_from_map);
        //corner_icp.set
        corner_icp.setInputSource(laserCloudCornerStack);
        corner_icp.setInputTarget(corner_localmap);

        corner_icp.align(*laserCloudCornerStack);

        if (corner_icp.hasConverged ())
        {
          std::cout << "\nICP has converged, score is " << corner_icp.getFitnessScore () << std::endl;
          corner_transformation_matrix = corner_icp.getFinalTransformation ().cast<double>();
          print4x4Matrix (corner_transformation_matrix);
        }

        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> surface_icp;
//        surface_icp.setSearchMethodSource(surface_curr_kdtree);
//        surface_icp.setSearchMethodTarget(kdtree_surf_from_map);

        surface_icp.setInputSource(laserCloudSurfStack);
        surface_icp.setInputTarget(in_laser_cloud_surf_from_map);

        surface_icp.align(*laserCloudSurfStack);

        if (surface_icp.hasConverged ())
        {
          std::cout << "\nICP has converged, score is " << surface_icp.getFitnessScore () << std::endl;
          surface_transformation_matrix = surface_icp.getFinalTransformation ().cast<double>();
          print4x4Matrix (surface_transformation_matrix);
        }

        //NDT

        //std::cout<<corner_localmap.points.size()<<std::endl;
//        writer.write("0.pcd",*corner_localmap);
        corner_localmap->width = 1;
        corner_localmap->height = corner_localmap->points.size();
        pcl::io::savePCDFileASCII("00.pcd",*corner_localmap);
        return 0;
    }
    Local_map()
    {
    }
};
