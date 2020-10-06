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
#include <Eigen/Dense>
#include <Eigen/StdVector>

//#include <eigen3/Eigen/Dense>
#include "common.h"
#include "tools_logger.hpp"
#include "tools_random.hpp"
#include<vector>
#include<algorithm>
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
#include <pcl/registration/ndt.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/common/transforms.h>
using namespace std;
class Local_map
{
    public:
    bool icp_continue=true;
    int numbers;
    int iter_num=0;
    float radius=0.3;

    pcl::PCDWriter writer;
    Eigen::Matrix4d corner_transformation_matrix = Eigen::Matrix4d::Identity ();
    Eigen::Matrix4d surface_transformation_matrix = Eigen::Matrix4d::Identity ();

    Eigen::Matrix4d corner_ndttrans = Eigen::Matrix4d::Identity ();
    Eigen::Matrix4d surface_ndttrans = Eigen::Matrix4d::Identity ();

    Eigen::Matrix4f init_ndtguess=Eigen::Matrix4f::Identity ();
    Eigen::Matrix3d rotation_matrix=Eigen::Matrix3d::Identity ();
    Eigen::Quaterniond quater;

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
    void
    print4x4Matrixf (const Eigen::Matrix4f & matrix)
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
        Eigen::Vector3d trans=Eigen::Vector3d(10,10,0.1);
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
        corner_icp.setInputTarget(in_laser_cloud_corner_from_map);

        corner_icp.align(*laserCloudCornerStack);

        if (corner_icp.hasConverged ())
        {
          std::cout << "\nICP has converged, score is " << corner_icp.getFitnessScore () << std::endl;
          corner_transformation_matrix = corner_icp.getFinalTransformation ().cast<double>();
          print4x4Matrix(corner_transformation_matrix);
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
        pcl::NormalDistributionsTransform<PointType, PointType> corner_ndt;
        pcl::NormalDistributionsTransform<PointType, PointType> surface_ndt;
        corner_ndt.setInputSource(laserCloudCornerStack);
        corner_ndt.setResolution(1.0);
        corner_ndt.setInputTarget(in_laser_cloud_corner_from_map);
        pcl::PointCloud<PointType>::Ptr ndt_cornerout (new pcl::PointCloud<PointType>);
        corner_ndt.align (*ndt_cornerout, init_ndtguess);
        if (corner_ndt.hasConverged ())
        {
          std::cout << "\nNDT has converged, score is " << corner_ndt.getFitnessScore () << std::endl;
          corner_ndttrans = corner_ndt.getFinalTransformation ().cast<double>();
          print4x4Matrix (corner_ndttrans);
        }

        surface_ndt.setInputSource(laserCloudSurfStack);
        surface_ndt.setResolution(1.0);
        surface_ndt.setInputTarget(in_laser_cloud_surf_from_map);
        pcl::PointCloud<PointType>::Ptr ndt_surfaceout (new pcl::PointCloud<PointType>);
        surface_ndt.align (*ndt_surfaceout, init_ndtguess);
        if (surface_ndt.hasConverged ())
        {
          std::cout << "\nNDT has converged, score is " << surface_ndt.getFitnessScore () << std::endl;
          surface_ndttrans = surface_ndt.getFinalTransformation ().cast<double>();
          print4x4Matrix (surface_ndttrans);
        }
        //std::cout<<corner_localmap.points.size()<<std::endl;
//        writer.write("0.pcd",*corner_localmap);
        //corner_localmap->width = 1;
        //corner_localmap->height = corner_localmap->points.size();
        //pcl::io::savePCDFileASCII("00.pcd",*corner_localmap);
        return 0;
    }
    int icp_findtrans(pcl::PointCloud< PointType >::Ptr in_laser_cloud_corner_from_map,
                      pcl::PointCloud< PointType >::Ptr in_laser_cloud_surf_from_map,
                      pcl::search::KdTree<PointType>::Ptr   kdtree_corner_from_map,
                      pcl::search::KdTree<PointType>::Ptr   kdtree_surf_from_map,
                      pcl::PointCloud< PointType >::Ptr laserCloudCornerStack,
                      pcl::PointCloud< PointType >::Ptr laserCloudSurfStack )
    {
        vector<double> fitness_score;
        vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>> trans_matrix;
        //Eigen::Matrix4d trans_test=Eigen::Matrix4d::Identity ();
        Eigen::Quaterniond quarter=Eigen::Quaterniond(-0.367821,0.0369714, -0.0113116, 0.929155).normalized();
        Eigen::Vector3d translation=Eigen::Vector3d(26.839, 7.90814, 0.063199);
        Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
        T.rotate(quarter.toRotationMatrix());
        T.pretranslate(translation);
        std::cout<<T.matrix()<<std::endl;
        //26.7186 8.3168 0.141909 -0.0207449 0.0154103 -0.996751 0.0762847
        ofstream OutFile("surfoutputpart_inialchange.txt"); //利用构造函数创建txt文本，并且打开该文本
        pcl::PointCloud<PointType>::Ptr corner_trans (new pcl::PointCloud<PointType>);
        //Eigen::Matrix4d corner_t_curr = Eigen::Matrix4d::Identity ();
        Eigen::Matrix4d corner_t_hist = Eigen::Matrix4d::Identity ();
        corner_t_hist=T.matrix();
        std::string cornerpath="/home/beihai/catkin_ws/src/loam_livox/pcd/laserCloudSurfStack/";
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> corner_icp;
        corner_icp.setInputTarget(in_laser_cloud_surf_from_map);

        corner_icp.setMaxCorrespondenceDistance(0.6);//points >0.5m ignored
        corner_icp.setMaximumIterations (50);
        corner_icp.setTransformationEpsilon (1e-8);
        corner_icp.setEuclideanFitnessEpsilon (0.05);

        for(int i=9961;i<12000;i++)
        {
            //print4x4Matrix (corner_t_curr);
            Eigen::Matrix4f init_icp=Eigen::Matrix4f::Identity ();
            pcl::PointCloud<PointType>::Ptr corner_curr (new pcl::PointCloud<PointType>);
            pcl::io::loadPCDFile<PointType> (cornerpath+std::to_string(i)+".pcd", *corner_curr);
            pcl::transformPointCloud (*corner_curr, *corner_trans, corner_t_hist);
            corner_icp.setInputSource(corner_trans);
            //corner_t_hist(2,3)=0.1;
            while(icp_continue)
            {
                //corner_icp.setRANSACOutlierRejectionThreshold(0.8);
                cerr<<"iter"<<iter_num<<endl;
                corner_icp.align(*corner_trans,init_icp);
                if (corner_icp.hasConverged())
                {
                    cerr<<"converged"<<endl;
                    fitness_score.push_back(corner_icp.getFitnessScore ());
                    cerr<<"pushback score ok"<<endl;

                    cerr<<trans_matrix.capacity()<<" "<<trans_matrix.size()<<endl;
                    Eigen::Matrix4d trans_test=corner_icp.getFinalTransformation().cast<double>();
                    trans_matrix.push_back(trans_test);
                    cerr<<trans_matrix.capacity()<<" "<<trans_matrix.size()<<endl;

                    cerr<<"pushback trans ok"<<endl;

                    //std::cout << "\nICP has converged, score is " << corner_icp.getFitnessScore () << std::endl;
                    //corner_t_curr = corner_icp.getFinalTransformation().cast<double>();
                    //print4x4Matrix (corner_t_curr);
                    if((corner_icp.getFitnessScore ()>0.05)&&(iter_num<8))
                    {
                        std::cout << "\nICP has converged, score is " << corner_icp.getFitnessScore () << std::endl;
                        std::cerr<<"numid"<<i<<endl;
                        switch (iter_num) {
                        case 0:
                            init_icp(0,3)=-0.2;
                            init_icp(1,3)=0;
                            iter_num++;
                            break;
                        case 1:
                            init_icp(0,3)=-0.2;
                            init_icp(1,3)=0.2;
                            iter_num++;
                            break;
                        case 2:
                            init_icp(0,3)=0;
                            init_icp(1,3)=0.2;
                            iter_num++;
                            break;
                        case 3:
                            init_icp(0,3)=0.2;
                            init_icp(1,3)=0.2;
                            iter_num++;
                            break;
                        case 4:
                            init_icp(0,3)=0.2;
                            init_icp(1,3)=0;
                            iter_num++;
                            break;
                        case 5:
                            init_icp(0,3)=0.2;
                            init_icp(1,3)=-0.2;
                            iter_num++;
                            break;
                        case 6:
                            init_icp(0,3)=0;
                            init_icp(1,3)=-0.2;
                            iter_num++;
                            break;
                        case 7:
                            init_icp(0,3)=-0.2;
                            init_icp(1,3)=-0.2;
                            iter_num++;
                            break;
                        default:
                            break;
                        }
                        print4x4Matrix(corner_t_hist);
                        icp_continue=true;
                    }
                    else
                    {
//                        std::cout << "\nICP has converged, score is " << corner_icp.getFitnessScore () << std::endl;
//                        std::cerr<<"numid"<<i<<endl;
                        cerr<<"end"<<endl;
                        icp_continue=false;
                    }
                }
            }

            iter_num=0;
            cerr<<trans_matrix.capacity()<<" "<<trans_matrix.size()<<endl;
            Eigen::Matrix4d corner_t_curr = trans_matrix[distance(begin(fitness_score),min_element(begin(fitness_score),end(fitness_score)))];
            cout<<distance(begin(fitness_score),min_element(begin(fitness_score),end(fitness_score)))<<endl;
            //corner_t_curr=trans_matrix[distance(begin(fitness_score),min_element(begin(fitness_score),end(fitness_score)))];
            icp_continue=true;
            fitness_score.clear();
            trans_matrix.clear();
            corner_t_hist=corner_t_curr*corner_t_hist;
            //rotation_matrix=corner_t_hist(0:2,0:2);
            for(int i=0;i<3;i++)
            {
                for(int j=0;j<3;j++)
                {
                    rotation_matrix(i,j)=corner_t_hist(i,j);
                }
            }
            quater=rotation_matrix;
            OutFile <<i<<" "<<corner_t_hist(0, 3)<<" "<<corner_t_hist (1, 3)<<" "<<corner_t_hist (2, 3)<<" "<<quater.x()<<" "<<quater.y()<<" "<<quater.z()<<" "<<quater.w()<<" "<<corner_icp.getFitnessScore()<<endl;


        }
        OutFile.close();            //关闭Test.txt文件
        return 0;
    }

    int icp_test(pcl::PointCloud< PointType >::Ptr in_laser_cloud_corner_from_map,
                      pcl::PointCloud< PointType >::Ptr in_laser_cloud_surf_from_map,
                      pcl::search::KdTree<PointType>::Ptr   kdtree_corner_from_map,
                      pcl::search::KdTree<PointType>::Ptr   kdtree_surf_from_map,
                      pcl::PointCloud< PointType >::Ptr laserCloudCornerStack,
                      pcl::PointCloud< PointType >::Ptr laserCloudSurfStack )
    {
        Eigen::Quaternionf quarter=Eigen::Quaternionf(0.0839445,0.0294684, -0.0326215, 0.995579).normalized();
        Eigen::Translation3f translation(26.0923-0.2, 7.49656+0.2, 0.183726);
        //25.9302 7.87992 0.156416 -0.0272266 0.0282275 -0.994084 -0.101287
        //26.0923 7.49656 0.183726 0.0294684 -0.0326215 0.995579 0.0839445 0.0329776
        Eigen::Affine3f T=translation*quarter.toRotationMatrix();
        Eigen::Matrix4f init_=Eigen::Matrix4f::Identity ();
        init_(2,3)=0.1;
        print4x4Matrixf(init_);
        Eigen::Matrix4f init_guess=T.matrix();

        std::string cornerpath="/home/beihai/catkin_ws/src/loam_livox/pcd/laserCloudSurfStack/10118.pcd";
        pcl::PointCloud<PointType>::Ptr corner_curr (new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr corner_trans (new pcl::PointCloud<PointType>);

        pcl::io::loadPCDFile<PointType> (cornerpath,*corner_curr);
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> corner_icp;
        corner_icp.setInputTarget(in_laser_cloud_surf_from_map);
        corner_icp.setInputSource(corner_curr);
        corner_icp.setMaxCorrespondenceDistance(0.6);//points >0.5m ignored
        corner_icp.setMaximumIterations (50);
        corner_icp.setTransformationEpsilon (1e-8);
        corner_icp.setEuclideanFitnessEpsilon (0.05);
        corner_icp.align(*corner_trans,init_guess);
        if (corner_icp.hasConverged ())
        {
            std::cout << "\nICP has converged, score is " << corner_icp.getFitnessScore () << std::endl;
            print4x4Matrix(corner_icp.getFinalTransformation ().cast<double>());
        }

        return 0;
    }
//    int ptp_icp_findtrans(pcl::PointCloud< PointType >::Ptr in_laser_cloud_corner_from_map,
//                      pcl::PointCloud< PointType >::Ptr in_laser_cloud_surf_from_map,
//                      pcl::search::KdTree<PointType>::Ptr   kdtree_corner_from_map,
//                      pcl::search::KdTree<PointType>::Ptr   kdtree_surf_from_map,
//                      pcl::PointCloud< PointType >::Ptr laserCloudCornerStack,
//                      pcl::PointCloud< PointType >::Ptr laserCloudSurfStack )
//    {

//        Eigen::Matrix4f corner_t_curr = Eigen::Matrix4f::Identity ();
//        std::string cornerpath="/home/beihai/catkin_ws/src/loam_livox/pcd/laserCloudSurfStack/";
//        //pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> corner_icp;
//        pcl::registration::TransformationEstimationPointToPlaneLLS< pcl::PointXYZI, pcl::PointXYZI > ptp_icp;
//        for(int i=1;i<100;i++)
//        {
//            //print4x4Matrix (corner_t_curr);
//            pcl::PointCloud<PointType>::Ptr corner_curr (new pcl::PointCloud<PointType>);
//            pcl::io::loadPCDFile<PointType> (cornerpath+std::to_string(i)+".pcd", *corner_curr);
//            ptp_icp.estimateRigidTransformation(*corner_curr,*in_laser_cloud_surf_from_map,corner_t_curr);
////            if (ptp_icp.hasConverged ())
////            {
////              std::cout << "\nICP has converged, score is " << ptp_icp.getFitnessScore () << std::endl;
////              corner_t_curr = ptp_icp.getFinalTransformation ();
////              //print4x4Matrixf (corner_t_curr);
////              printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", corner_t_curr (0, 3), corner_t_curr (1, 3), corner_t_curr (2, 3));


////            }
//        }
//        return 0;
//    }
    Local_map()
    {
    }
};
