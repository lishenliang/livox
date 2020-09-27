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
#include<ctime>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include "common.h"
#include "tools_logger.hpp"
#include "tools_random.hpp"

#include "pcl_tools.hpp"
#include "ceres_icp.hpp"
#include "local_map.hpp"
// PCL specific includes
//#include <pcl/ros/conversions.h>
#include<pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#define CORNER_MIN_MAP_NUM 0
#define SURFACE_MIN_MAP_NUM 50

using namespace std;
using namespace Common_tools;
using namespace PCL_TOOLS;

int m_para_icp_max_iterations = 200;
int m_maximum_allow_residual_block = 1e5;
int m_if_motion_deblur = 1 ;
int line_search_num = 5;
int plane_search_num = 5;

int IF_LINE_FEATURE_CHECK=0;
int IF_PLANE_FEATURE_CHECK=0;
int ICP_PLANE = 1;
int ICP_LINE= 1;
int    m_para_cere_max_iterations = 100;
int    m_para_cere_prerun_times = 2;

float  m_minimum_pt_time_stamp = 0;
float  m_maximum_pt_time_stamp = 1.0;
double m_maximum_dis_line_for_match = 2.0;
double m_maximum_dis_plane_for_match = 50.0;
double m_inlier_ratio = 0.80;
double m_inlier_threshold;
double m_inliner_dis = 0.02;
double m_angular_diff = 0;
double m_t_diff = 0;
double               m_minimum_icp_R_diff = 0.01;
double               m_minimum_icp_T_diff = 0.01;
double endtime;
float  m_para_max_angular_rate = 200.0 / 50.0; // max angular rate = 90.0 /50.0 deg/s
float  m_para_max_speed = 100.0 / 50.0;        // max speed = 10 m/s
float  m_last_time_stamp = 0;
float  m_max_final_cost = 100.0;

Common_tools::Random_generator_float<float> m_rand_float;
ceres::Solver::Summary summary;
ceres::Solver::Summary               m_final_opt_summary;
Eigen::Quaterniond m_q_w_curr, m_q_w_last;
Eigen::Vector3d m_t_w_curr, m_t_w_last;
ceres::LinearSolverType slover_type = ceres::DENSE_SCHUR; // SPARSE_NORMAL_CHOLESKY | DENSE_QR | DENSE_SCHUR

double m_para_buffer_RT[ 7 ] = { 0, 0, 0, 1, 0, 0, 0 };
double m_para_buffer_RT_last[ 7 ] = { 0, 0, 0, 1, 0, 0, 0 };
double m_para_buffer_incremental[ 7 ] = { 0, 0, 0, 1, 0, 0, 0 };
pcl::KdTreeFLANN<PointType> m_kdtree_corner_from_map;
pcl::KdTreeFLANN<PointType> m_kdtree_surf_from_map;

clock_t t_start,t_end;
void set_ceres_solver_bound( ceres::Problem &problem ,double * para_buffer_RT )
{
    for ( unsigned int i = 0; i < 3; i++ )
    {

        problem.SetParameterLowerBound( para_buffer_RT + 4, i, -m_para_max_speed );
        problem.SetParameterUpperBound( para_buffer_RT + 4, i, +m_para_max_speed );
    }
}

double compute_inlier_residual_threshold( std::vector< double > residuals, double ratio )
{
    std::set< double > dis_vec;
    for ( size_t i = 0; i < ( size_t )( residuals.size() / 3 ); i++ )
    {
        dis_vec.insert( fabs( residuals[ 3 * i + 0 ] ) + fabs( residuals[ 3 * i + 1 ] ) + fabs( residuals[ 3 * i + 2 ] ) );
    }
    return *( std::next( dis_vec.begin(), ( int ) ( ( ratio ) * dis_vec.size() ) ) );
}


float refine_blur( float in_blur, const float &min_blur, const float &max_blur )
{
    float res = 1.0;
    if ( m_if_motion_deblur )
    {
        res = ( in_blur - min_blur ) / ( max_blur - min_blur );
        if ( !std::isfinite( res ) || res > 1.0)
            return 1.0;
        else
            return res;
    }

    return res;
}

void pointAssociateToMap( PointType const *const pi, PointType *const po,
                            double interpolate_s = 1.0, int if_undistore = 0 )
{
    Eigen::Vector3d point_curr( pi->x, pi->y, pi->z );
    Eigen::Vector3d point_w;
    if ( if_undistore == 0 || interpolate_s == 1.0 )
    {
        point_w = m_q_w_curr * point_curr + m_t_w_curr;
    }
//    else
//    {
//        if ( interpolate_s > 1.0 || interpolate_s < 0.0 )
//        {
//            screen_printf( "Input interpolate_s = %.5f\r\n", interpolate_s );
//        }

//        if ( 1 ) // Using rodrigues for fast compute.
//        {
//            //https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
//            Eigen::Vector3d             interpolate_T = m_t_w_incre * ( interpolate_s * BLUR_SCALE );
//            double                      interpolate_R_theta = m_interpolatation_theta * interpolate_s;
//            Eigen::Matrix<double, 3, 3> interpolate_R_mat;

//            interpolate_R_mat = Eigen::Matrix3d::Identity() + sin( interpolate_R_theta ) * m_interpolatation_omega_hat + ( 1 - cos( interpolate_R_theta ) ) * m_interpolatation_omega_hat_sq2;
//            point_w = m_q_w_last * ( interpolate_R_mat * point_curr + interpolate_T ) + m_t_w_last;
//        }
//        else
//        {
//            Eigen::Quaterniond interpolate_q = m_q_I.slerp( interpolate_s * BLUR_SCALE, m_q_w_incre );
//            Eigen::Vector3d    interpolate_T = m_t_w_incre * ( interpolate_s * BLUR_SCALE );
//            point_w = m_q_w_last * ( interpolate_q * point_curr + interpolate_T ) + m_t_w_last;
//        }
//    }

    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}


int find_out_incremental_transfrom( pcl::PointCloud< PointType >::Ptr in_laser_cloud_corner_from_map,
                                      pcl::PointCloud< PointType >::Ptr in_laser_cloud_surf_from_map,
                                      pcl::KdTreeFLANN< PointType > &   kdtree_corner_from_map,
                                      pcl::KdTreeFLANN< PointType > &   kdtree_surf_from_map,
                                      pcl::PointCloud< PointType >::Ptr laserCloudCornerStack,
                                      pcl::PointCloud< PointType >::Ptr laserCloudSurfStack )
{
    Eigen::Map<Eigen::Quaterniond> q_w_incre = Eigen::Map<Eigen::Quaterniond>(m_para_buffer_incremental );
    Eigen::Map<Eigen::Vector3d> t_w_incre = Eigen::Map<Eigen::Vector3d>( m_para_buffer_incremental + 4 );
    m_kdtree_corner_from_map = kdtree_corner_from_map;
    m_kdtree_surf_from_map = kdtree_surf_from_map;

    pcl::PointCloud<PointType> laser_cloud_corner_from_map =  *in_laser_cloud_corner_from_map;
    pcl::PointCloud<PointType> laser_cloud_surf_from_map =  *in_laser_cloud_surf_from_map;

    int laserCloudCornerFromMapNum = laser_cloud_corner_from_map.points.size();
    int laserCloudSurfFromMapNum = laser_cloud_surf_from_map.points.size();
    int laser_corner_pt_num = laserCloudCornerStack->points.size();
    int laser_surface_pt_num = laserCloudSurfStack->points.size();
    cerr<<"corner number is : "<<laser_corner_pt_num<<endl;
    cerr<<"surface number is : "<<laser_surface_pt_num<<endl;

    int                    surf_avail_num = 0;
    int                    corner_avail_num = 0;
    float                  minimize_cost = summary.final_cost ;
    PointType              pointOri, pointSel;
    int                    corner_rejection_num = 0;
    int                    surface_rejecetion_num = 0;
    int                    if_undistore_in_matching = 1;
    if ( laserCloudCornerFromMapNum > CORNER_MIN_MAP_NUM && laserCloudSurfFromMapNum > SURFACE_MIN_MAP_NUM )
    {

        Eigen::Quaterniond q_last_optimize( 1.f, 0.f, 0.f, 0.f );
        Eigen::Vector3d    t_last_optimize( 0.f, 0.f, 0.f );
        int                iterCount = 0;

        std::vector<int>   m_point_search_Idx;
        std::vector<float> m_point_search_sq_dis;


        for ( iterCount = 0; iterCount < m_para_icp_max_iterations; iterCount++ )
        {
            cerr<<"iteration "<<iterCount+1<<endl;
            m_point_search_Idx.clear();
            m_point_search_sq_dis.clear();
            corner_avail_num = 0;
            surf_avail_num = 0;
            corner_rejection_num = 0;
            surface_rejecetion_num = 0;

            ceres::LossFunction *               loss_function = new ceres::HuberLoss( 0.1 );
            ceres::LocalParameterization *      q_parameterization = new ceres::EigenQuaternionParameterization();
            ceres::Problem::Options             problem_options;
            ceres::ResidualBlockId              block_id;
            ceres::Problem                      problem( problem_options );
            std::vector<ceres::ResidualBlockId> residual_block_ids;

            problem.AddParameterBlock( m_para_buffer_incremental, 4, q_parameterization );
            problem.AddParameterBlock( m_para_buffer_incremental + 4, 3 );
            for ( int i = 0; i < laser_corner_pt_num; i++ )
            {

                if ( laser_corner_pt_num > 2 * m_maximum_allow_residual_block )
               {
                   if(m_rand_float.rand_uniform() * laser_corner_pt_num >  2 * m_maximum_allow_residual_block)
                   {
                       continue;
                   }
                }
                   pointOri = laserCloudCornerStack->points[ i ];
                   //printf_line;
                   if ( (!std::isfinite( pointOri.x )) ||
                        (!std::isfinite( pointOri.y )) ||
                        (!std::isfinite( pointOri.z )) )
                       continue;
                   pointAssociateToMap( &pointOri, &pointSel,1.0,if_undistore_in_matching);

                   if(m_kdtree_corner_from_map.nearestKSearch( pointSel, line_search_num, m_point_search_Idx, m_point_search_sq_dis ) != line_search_num)
                   {
                     cerr<<"erro"<<endl;

                     continue;
                   }

                   if ( m_point_search_sq_dis[ line_search_num - 1 ] < m_maximum_dis_line_for_match )
                   {
                       bool                         line_is_avail = true;
                       std::vector<Eigen::Vector3d> nearCorners;
                       Eigen::Vector3d              center( 0, 0, 0 );
                       if ( IF_LINE_FEATURE_CHECK )
                       {
                           for ( int j = 0; j < line_search_num; j++ )
                           {
                               Eigen::Vector3d tmp( laser_cloud_corner_from_map.points[ m_point_search_Idx[ j ] ].x,
                                                    laser_cloud_corner_from_map.points[ m_point_search_Idx[ j ] ].y,
                                                    laser_cloud_corner_from_map.points[ m_point_search_Idx[ j ] ].z );
                               center = center + tmp;
                               nearCorners.push_back( tmp );
                           }

                           center = center / ( ( float ) line_search_num );

                           Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();

                           for ( int j = 0; j < line_search_num; j++ )
                           {
                               Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[ j ] - center;
                               covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                           }

                           Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes( covMat );

                           // note Eigen library sort eigenvalues in increasing order

                           if ( saes.eigenvalues()[ 2 ] > 3 * saes.eigenvalues()[ 1 ] )
                           {
                               line_is_avail = true;
                           }
                           else
                           {
                               line_is_avail = false;
                               cerr<<"line ia false"<<endl;
                           }
                       }
                       Eigen::Vector3d curr_point( pointOri.x, pointOri.y, pointOri.z );
                       if ( line_is_avail )
                       {
                           if ( ICP_LINE )
                           {

                               ceres::CostFunction *cost_function;
                               auto                 pt_1 = pcl_pt_to_eigend( laser_cloud_corner_from_map.points[ m_point_search_Idx[ 0 ] ] );
                               //error
                               auto                 pt_2 = pcl_pt_to_eigend( laser_cloud_corner_from_map.points[ m_point_search_Idx[ 1 ] ] );

                               if((pt_1 -pt_2).norm() < 0.0001)
                                 {
                                   continue;
                                 }

                               if ( m_if_motion_deblur )
                               {

                                   //printf_line;
                                   //cost_function = ceres_icp_point2point_mb<double>::Create(curr_point,pt_1);
                                   cost_function = ceres_icp_point2line_mb<double>::Create( curr_point,
                                                                                         pt_1,
                                                                                         pt_2,
                                                                                         refine_blur( pointOri.intensity, m_minimum_pt_time_stamp, m_maximum_pt_time_stamp ) * 1.0,
                                                                                         Eigen::Matrix<double, 4, 1>( m_q_w_last.w(), m_q_w_last.x(), m_q_w_last.y(), m_q_w_last.z() ),
                                                                                         m_t_w_last ); //pointOri.intensity );
                               }
                               else
                               {
                                   //cost_function = ceres_icp_point2point_mb<double>::Create(curr_point,pt_1);
                                   cost_function = ceres_icp_point2line<double>::Create( curr_point,
                                                                                         pt_1,
                                                                                         pt_2,
                                                                                         Eigen::Matrix<double, 4, 1>( m_q_w_last.w(), m_q_w_last.x(), m_q_w_last.y(), m_q_w_last.z() ),
                                                                                         m_t_w_last );
                               }
                               //printf_line;
                               block_id = problem.AddResidualBlock( cost_function, loss_function, m_para_buffer_incremental, m_para_buffer_incremental + 4 );
                               residual_block_ids.push_back( block_id );
                               corner_avail_num++;
                           }
                       }
                       else
                       {
                           corner_rejection_num++;
                       }
                   }

            }


            for ( int i = 0; i < laser_surface_pt_num; i++ )
            {

                if ( laser_surface_pt_num > 2 * m_maximum_allow_residual_block )
                {
                    if(m_rand_float.rand_uniform() * laser_surface_pt_num >  2 * m_maximum_allow_residual_block)
                    {
                        continue;
                    }
                }

                pointOri = laserCloudSurfStack->points[ i ];
                int planeValid = true;
                pointAssociateToMap( &pointOri, &pointSel,1.0,if_undistore_in_matching);
                //printf_line;
                m_kdtree_surf_from_map.nearestKSearch( pointSel, plane_search_num, m_point_search_Idx, m_point_search_sq_dis );

                if ( m_point_search_sq_dis[ plane_search_num - 1 ] < m_maximum_dis_plane_for_match )
                {
                    std::vector<Eigen::Vector3d> nearCorners;
                    Eigen::Vector3d              center( 0, 0, 0 );
                    if ( IF_PLANE_FEATURE_CHECK )
                    {
                        for ( int j = 0; j < plane_search_num; j++ )
                        {
                            Eigen::Vector3d tmp( laser_cloud_corner_from_map.points[ m_point_search_Idx[ j ] ].x,
                                                 laser_cloud_corner_from_map.points[ m_point_search_Idx[ j ] ].y,
                                                 laser_cloud_corner_from_map.points[ m_point_search_Idx[ j ] ].z );
                            center = center + tmp;
                            nearCorners.push_back( tmp );
                        }

                        center = center / ( float ) ( plane_search_num );

                        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();

                        for ( int j = 0; j < plane_search_num; j++ )
                        {
                            Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[ j ] - center;
                            covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                        }

                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes( covMat );

                        if ( ( saes.eigenvalues()[ 2 ] > 3 * saes.eigenvalues()[ 0 ] ) &&
                             ( saes.eigenvalues()[ 2 ] < 10 * saes.eigenvalues()[ 1 ] ) )
                        {
                            planeValid = true;
                        }
                        else
                        {
                            planeValid = false;
                            cerr<<"plane ia false"<<endl;

                        }
                    }

                    Eigen::Vector3d curr_point( pointOri.x, pointOri.y, pointOri.z );

                    if ( planeValid )
                    {
                        if ( ICP_PLANE )
                        {
                            ceres::CostFunction *cost_function;
                            auto                 pt_1 = pcl_pt_to_eigend( laser_cloud_surf_from_map.points[ m_point_search_Idx[ 0 ] ] );
                            //error
                            auto                 pt_2 = pcl_pt_to_eigend( laser_cloud_surf_from_map.points[ m_point_search_Idx[ 1 ] ] );
                            if ( m_if_motion_deblur )
                            {
                                //cost_function = ceres_icp_point2point_mb<double>::Create(curr_point,pt_1);

                              cost_function = ceres_icp_point2plane_mb<double>::Create(
                                  curr_point,
                                  pcl_pt_to_eigend( laser_cloud_surf_from_map.points[ m_point_search_Idx[ 0 ] ] ),
                                  pcl_pt_to_eigend( laser_cloud_surf_from_map.points[ m_point_search_Idx[ plane_search_num / 2 ] ] ),
                                  pcl_pt_to_eigend( laser_cloud_surf_from_map.points[ m_point_search_Idx[ plane_search_num - 1 ] ] ),
                                  refine_blur( pointOri.intensity, m_minimum_pt_time_stamp, m_maximum_pt_time_stamp ) * 1.0,
                                  Eigen::Matrix<double, 4, 1>( m_q_w_last.w(), m_q_w_last.x(), m_q_w_last.y(), m_q_w_last.z() ),
                                  m_t_w_last );

                            }
                            else
                            {
                                //cost_function = ceres_icp_point2point_mb<double>::Create(curr_point,pt_1);

                                cost_function = ceres_icp_point2plane<double>::Create(
                                    curr_point,
                                    pcl_pt_to_eigend( laser_cloud_surf_from_map.points[ m_point_search_Idx[ 0 ] ] ),
                                    pcl_pt_to_eigend( laser_cloud_surf_from_map.points[ m_point_search_Idx[ plane_search_num / 2 ] ] ),
                                    pcl_pt_to_eigend( laser_cloud_surf_from_map.points[ m_point_search_Idx[ plane_search_num - 1 ] ] ),
                                    Eigen::Matrix<double, 4, 1>( m_q_w_last.w(), m_q_w_last.x(), m_q_w_last.y(), m_q_w_last.z() ),
                                    m_t_w_last ); //pointOri.intensity );
                            }
                            block_id = problem.AddResidualBlock( cost_function, loss_function, m_para_buffer_incremental, m_para_buffer_incremental + 4 );
                            residual_block_ids.push_back( block_id );
                        }
                        surf_avail_num++;
                    }
                    else
                    {
                        surface_rejecetion_num++;
                    }
                }
            }

            std::vector< ceres::ResidualBlockId > residual_block_ids_temp;
            residual_block_ids_temp.reserve( residual_block_ids.size() );

            // Drop some of the residual to guaruntee the real time performance.
            if ( residual_block_ids.size() > (size_t) m_maximum_allow_residual_block )
            {
                residual_block_ids_temp.clear();

                float  threshold_to_reserve = ( float ) m_maximum_allow_residual_block / ( float ) residual_block_ids.size();
                float *probability_to_drop = m_rand_float.rand_array_uniform( 0, 1.0, residual_block_ids.size() );
                cerr << "Number of residual blocks too Large, drop them to " << m_maximum_allow_residual_block << endl;
                for ( size_t i = 0; i < residual_block_ids.size(); i++ )
                {
                    if ( probability_to_drop[ i ] > threshold_to_reserve )
                    {
                        problem.RemoveResidualBlock( residual_block_ids[ i ] );
                    }
                    else
                    {
                        residual_block_ids_temp.push_back( residual_block_ids[ i ] );
                    }
                }
                residual_block_ids = residual_block_ids_temp;
                delete probability_to_drop;
            }

            ceres::Solver::Options options;

            // If the number of residual block too Large, randomly drop some of them to guarentee the real-time perfromance.
            for ( size_t ii = 0; ii < 1; ii++ )
            {
                options.linear_solver_type = slover_type;
                options.max_num_iterations = m_para_cere_max_iterations;
                options.max_num_iterations = m_para_cere_prerun_times;
                options.minimizer_progress_to_stdout = false;
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e-10;
                //options.function_tolerance = 1e-100; // default 1e-6

                set_ceres_solver_bound( problem, m_para_buffer_incremental );
                ceres::Solve( options, &problem, &summary );

                residual_block_ids_temp.clear();
                ceres::Problem::EvaluateOptions eval_options;
                eval_options.residual_blocks = residual_block_ids;
                double              total_cost = 0.0;
                std::vector<double> residuals;
                problem.Evaluate( eval_options, &total_cost, &residuals, nullptr, nullptr );
                //double avr_cost = total_cost / residual_block_ids.size();

                double m_inliner_ratio_threshold = compute_inlier_residual_threshold( residuals, m_inlier_ratio );
                m_inlier_threshold = std::max( m_inliner_dis, m_inliner_ratio_threshold );
                //screen_out << "Inlier threshold is: " << m_inlier_final_threshold << endl;
                for ( unsigned int i = 0; i < residual_block_ids.size(); i++ )
                {
                    if ( ( fabs( residuals[ 3 * i + 0 ] ) + fabs( residuals[ 3 * i + 1 ] ) + fabs( residuals[ 3 * i + 2 ] ) ) > m_inlier_threshold ) // std::min( 1.0, 10 * avr_cost )
                    {
                        //screen_out << "Remove outliers, drop id = " << (void *)residual_block_ids[ i ] <<endl;
                        problem.RemoveResidualBlock( residual_block_ids[ i ] );
                    }
                    else
                    {
                        residual_block_ids_temp.push_back( residual_block_ids[ i ] );
                    }
                }
                residual_block_ids = residual_block_ids_temp;
            }

            options.linear_solver_type = slover_type;
            options.max_num_iterations = m_para_cere_max_iterations;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-10;

            t_start=clock();
            set_ceres_solver_bound( problem, m_para_buffer_incremental );
            ceres::Solve( options, &problem, &summary );
            t_end=clock();
            endtime=(double)(t_end-t_start)/CLOCKS_PER_SEC;
            cerr<<"soulve once time:"<<endtime*1000<<"ms"<<endl;	//ms为单位

            //cout<<summary.BriefReport()<<"\n";
            m_t_w_curr = m_q_w_last * t_w_incre + m_t_w_last;
            m_q_w_curr = m_q_w_last * q_w_incre;

            m_angular_diff = ( float ) m_q_w_curr.angularDistance( m_q_w_last ) * 57.3;
            m_t_diff = ( m_t_w_curr - m_t_w_last ).norm();
            minimize_cost = summary.final_cost;
            cerr<<"x:"<<m_t_w_curr[0]<<" y:"<<m_t_w_curr[1]<<" z:"<<m_t_w_curr[2]<<endl;
            cerr<<"Q:"<<m_q_w_curr.coeffs().transpose()<<endl;
            if ( q_last_optimize.angularDistance( q_w_incre ) < 57.3 * m_minimum_icp_R_diff &&
                 ( t_last_optimize - t_w_incre ).norm() < m_minimum_icp_T_diff )
            {
                cerr << "----- Terminate, iteration times  = " << iterCount << "-----" << endl;
                cerr<<"x: "<<m_t_w_curr[0]<<"y: "<<m_t_w_curr[1]<<"z: "<<m_t_w_curr[2]<<endl;
                break;
            }
            else
            {
                q_last_optimize = q_w_incre;
                t_last_optimize = t_w_incre;
            }
        }
            m_inlier_threshold = m_inlier_threshold* summary.final_cost/ summary.initial_cost; //
            if ( m_angular_diff > m_para_max_angular_rate || minimize_cost > m_max_final_cost )
            {
                for ( int i = 0; i < 7; i++ )
                {
                    m_para_buffer_RT[ i ] = m_para_buffer_RT_last[ i ];
                }
                m_last_time_stamp = m_minimum_pt_time_stamp;
                m_q_w_curr = m_q_w_last;
                m_t_w_curr = m_t_w_last;
                return 0;
            }
            m_final_opt_summary = summary;
    }
    else
    {
        cerr << "time Map corner and surf num are not enough" << std::endl;
    }

    return 1;

}

int main (int argc, char** argv)
{
// Initialize ROS
    ros::init (argc, argv, "map_creator");
    ros::NodeHandle nh;
    
// Create a ROS subscriber for the input point cloud
    //ros::Subscriber corner_sub = nh.subscribe ("pc2_corners", 5, corner_Callback);
    //ros::Subscriber surface_sub = nh.subscribe ("pc2_surface", 5, surface_Callback);

// Create a ROS publisher for the output point cloud
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry> ("pose", 1);
    //map_pub= nh.advertise<nav_msgs::OccupancyGrid> ("map", 2,true);
// Spin

    pcl::PointCloud<PointType>::Ptr cornerstack (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr surfacestack (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr corner (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr surface (new pcl::PointCloud<PointType>);

    string cornermap_dir="/home/beihai/my_workspace/devel/lib/livox_aftermaped/CYT02_cornermap01.pcd";
    string surfacemap_dir="/home/beihai/my_workspace/devel/lib/livox_aftermaped/CYT02_surfacemap01.pcd";
    string corner_dir="corner200.pcd";
    string surface_dir="surface200.pcd";

    t_start=clock();
    pcl::io::loadPCDFile<PointType> (cornermap_dir, *cornerstack);
    pcl::io::loadPCDFile<PointType> (surfacemap_dir, *surfacestack);
    pcl::io::loadPCDFile<PointType> (corner_dir, *corner);
    pcl::io::loadPCDFile<PointType> (surface_dir, *surface);
    t_end=clock();
    endtime=(double)(t_end-t_start)/CLOCKS_PER_SEC;
    cerr<<"read time:"<<endtime*1000<<"ms"<<endl;	//ms为单位

    std::cerr<<"corner_number: "<<cornerstack->points.size()<<std::endl;
    std::cerr<<"surface_number: "<<surfacestack->points.size()<<std::endl;

    pcl::search::KdTree<PointType>::Ptr corner_kdtree( new pcl::search::KdTree<PointType>);
    pcl::search::KdTree<PointType>::Ptr surface_kdtree( new pcl::search::KdTree<PointType>);



    t_start=clock();
    corner_kdtree->setInputCloud (cornerstack);
    surface_kdtree->setInputCloud (surfacestack);

    t_end=clock();
    endtime=(double)(t_end-t_start)/CLOCKS_PER_SEC;
    cerr<<"kdtree time:"<<endtime*1000<<"ms"<<endl;	//ms为单位

    //pcl::PointXYZ searchPoint;
    t_start=clock();
    //find_out_incremental_transfrom(cornerstack,surfacestack,corner_kdtree,surface_kdtree,corner,surface);
    Local_map local;
    local.local_map_creater(cornerstack,surfacestack,corner_kdtree,surface_kdtree,corner,surface);
    t_end=clock();
    endtime=(double)(t_end-t_start)/CLOCKS_PER_SEC;
    cerr<<"calculate time:"<<endtime*1000<<"ms"<<endl;	//ms为单位
    ros::MultiThreadedSpinner spinner(2); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
    //ros::spin ();
}
