#ifndef FAST_LIO_SAM_MAIN_H
#define FAST_LIO_SAM_MAIN_H

///// coded headers
#include "utilities.h"
///// common headers
#include <time.h>
#include <math.h>
#include <cmath>
#include <chrono> //time check
#include <vector>
#include <mutex>
#include <string>
#include <utility> // pair, make_pair
///// ROS
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler
#include <tf/transform_datatypes.h> // createQuaternionFromRPY
#include <tf_conversions/tf_eigen.h> // tf <-> eigen
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
///// PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <pcl/common/transforms.h> //transformPointCloud
#include <pcl/conversions.h> //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/filters/voxel_grid.h> //voxelgrid
#include <pcl/registration/gicp.h> //gicp
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
///// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

using namespace std;
using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> odom_pcd_sync_pol;


////////////////////////////////////////////////////////////////////////////////////////////////////
struct pose_pcd
{
  pcl::PointCloud<pcl::PointXYZI> pcd;
  gtsam::Pose3 pose_gtsam;
  Eigen::Matrix4d pose_eig = Eigen::Matrix4d::Identity();
  double timestamp;
  pose_pcd(){};
  pose_pcd(const nav_msgs::Odometry &odom_in, const sensor_msgs::PointCloud2 &pcd_in);
};
////////////////////////////////////////////////////////////////////////////////////////////////////
class FAST_LIO_SAM_CLASS
{
  private:
    ///// basic params
    string m_map_frame;
    ///// shared data - odom and pcd
    mutex m_pose_pcd_mutex, m_corrected_esti_mutex, m_vis_vars_mutex, m_odom_delta_mutex, m_pcd_map_mutex;
    bool m_init=false;
    pose_pcd m_current_keyframe;
    vector<pose_pcd> m_keyframes;
    Eigen::Matrix4d m_last_corrected_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d m_odom_delta = Eigen::Matrix4d::Identity();
    ///// graph and values
    shared_ptr<gtsam::ISAM2> m_isam_handler = nullptr;
    gtsam::NonlinearFactorGraph m_gtsam_graph;
    gtsam::Values m_init_esti;
    gtsam::Values m_corrected_esti;
    double m_keyframe_thr;
    ///// loop
    pcl::VoxelGrid<pcl::PointXYZI> m_voxelgrid;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> m_gicp;
    double m_gicp_score_thr;
    double m_loop_det_radi;
    double m_loop_det_tdiff_thr;
    vector<pair<int, int>> m_loop_idx_pairs; //for vis
    bool m_loop_added_flag = false; //for vis
    ///// visualize
    pcl::PointCloud<pcl::PointXYZ> m_odoms, m_corrected_odoms;
    nav_msgs::Path m_odom_path, m_corrected_path;
    pcl::PointCloud<pcl::PointXYZI> m_corrected_map;
    ///// ros
    ros::NodeHandle m_nh;
    ros::Publisher m_corrected_odom_pub, m_corrected_path_pub, m_odom_pub, m_path_pub;
    ros::Publisher m_corrected_current_pcd_pub, m_corrected_pcd_map_pub, m_loop_detection_pub;
    ros::Publisher m_realtime_pose_pub;
    ros::Timer m_pgo_timer;
    // odom, pcd sync subscriber
    shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> m_sub_odom_pcd_sync = nullptr;
    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> m_sub_odom = nullptr;
    shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> m_sub_pcd = nullptr;

    ///// functions
  public:
    FAST_LIO_SAM_CLASS(const ros::NodeHandle& n_private);
  private:
    //methods
    void update_vis_vars(const pose_pcd &pose_pcd_in);
    inline bool check_if_keyframe(const pose_pcd &pose_pcd_in);
    visualization_msgs::Marker get_loop_markers();
    //cb
    void odom_pcd_cb(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg);
    void pgo_timer_func(const ros::TimerEvent& event);
};




///////////////////////////////////// can be separated into .cpp files
pose_pcd::pose_pcd(const nav_msgs::Odometry &odom_in, const sensor_msgs::PointCloud2 &pcd_in)
{
  timestamp = odom_in.header.stamp.toSec();
  pcl::fromROSMsg(pcd_in, pcd);
  tf::Quaternion q_(odom_in.pose.pose.orientation.x, odom_in.pose.pose.orientation.y, odom_in.pose.pose.orientation.z, odom_in.pose.pose.orientation.w);
  tf::Matrix3x3 m_(q_);
  Eigen::Matrix3d tmp_rot_mat_;
  tf::matrixTFToEigen(m_, tmp_rot_mat_);
  pose_eig.block<3, 3>(0, 0) = tmp_rot_mat_;
  pose_eig(0, 3) = odom_in.pose.pose.position.x;
  pose_eig(1, 3) = odom_in.pose.pose.position.y;
  pose_eig(2, 3) = odom_in.pose.pose.position.z;
  pose_gtsam = gtsam::Pose3(gtsam::Rot3::Quaternion(odom_in.pose.pose.orientation.w, odom_in.pose.pose.orientation.x, odom_in.pose.pose.orientation.y, odom_in.pose.pose.orientation.z), 
                            gtsam::Point3(odom_in.pose.pose.position.x, odom_in.pose.pose.position.y, odom_in.pose.pose.position.z));
}
FAST_LIO_SAM_CLASS::FAST_LIO_SAM_CLASS(const ros::NodeHandle& n_private) : m_nh(n_private)
{
  ////// ROS params
  // temp vars
  double pgo_update_hz_;
  // get params
  m_nh.param<string>("/map_frame", m_map_frame, "map");
  m_nh.param<double>("/keyframe_threshold", m_keyframe_thr, 1.0);
  m_nh.param<double>("/loop_detection_radius", m_loop_det_radi, 15.0);
  m_nh.param<double>("/loop_detection_timediff_threshold", m_loop_det_tdiff_thr, 10.0);
  m_nh.param<double>("/gicp_score_threshold", m_gicp_score_thr, 10.0);
  m_nh.param<double>("/pgo_update_hz", pgo_update_hz_, 3.0);

  ////// GTSAM init
  gtsam::ISAM2Params isam_params_;
  isam_params_.relinearizeThreshold = 0.01;
  isam_params_.relinearizeSkip = 1;
  m_isam_handler = std::make_shared<gtsam::ISAM2>(isam_params_);
  ////// loop init
  m_voxelgrid.setLeafSize(0.3, 0.3, 0.3);
  m_gicp.setTransformationEpsilon(0.1);
  m_gicp.setMaxCorrespondenceDistance(m_loop_det_radi*1.5);
  m_gicp.setMaximumIterations(100);
  m_gicp.setEuclideanFitnessEpsilon(1);

  ////// ROS things
  m_odom_path.header.frame_id = m_map_frame;
  m_corrected_path.header.frame_id = m_map_frame;
  // publishers
  m_odom_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/ori_odom", 10, true);
  m_path_pub = m_nh.advertise<nav_msgs::Path>("/ori_path", 10, true);
  m_corrected_odom_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/corrected_odom", 10, true);
  m_corrected_path_pub = m_nh.advertise<nav_msgs::Path>("/corrected_path", 10, true);
  m_corrected_pcd_map_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/corrected_map", 10, true);
  m_corrected_current_pcd_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/corrected_current_pcd", 10, true);
  m_loop_detection_pub = m_nh.advertise<visualization_msgs::Marker>("/loop_detection", 10, true);
  m_realtime_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/pose_stamped", 10);
  // subscribers
  m_sub_odom = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(m_nh, "/Odometry", 10);
  m_sub_pcd = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(m_nh, "/cloud_registered", 10);
  m_sub_odom_pcd_sync = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *m_sub_odom, *m_sub_pcd);
  m_sub_odom_pcd_sync->registerCallback(boost::bind(&FAST_LIO_SAM_CLASS::odom_pcd_cb, this, _1, _2));
  // Timers at the end
  m_pgo_timer = m_nh.createTimer(ros::Duration(1/pgo_update_hz_), &FAST_LIO_SAM_CLASS::pgo_timer_func, this);
  
  ROS_WARN("Main class, starting node...");
}

void FAST_LIO_SAM_CLASS::odom_pcd_cb(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg)
{
  Eigen::Matrix4d last_odom_tf_ = m_current_keyframe.pose_eig;
  {
    lock_guard<mutex> lock(m_pose_pcd_mutex);
    m_current_keyframe = pose_pcd(*odom_msg, *pcd_msg);
  }
  if (!m_init) //// init only once
  {
    //others
    m_keyframes.push_back(m_current_keyframe);
    m_corrected_map += m_keyframes.back().pcd;
    {
      lock_guard<mutex> lock(m_vis_vars_mutex);
      update_vis_vars(m_keyframes.back());
    }
    //graph
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4).finished()); // rad*rad, meter*meter
    m_gtsam_graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, m_keyframes.back().pose_gtsam, prior_noise_));
    m_init_esti.insert(0, m_keyframes.back().pose_gtsam);
    m_init = true;
  }
  else // realtime pose = last corrected odom * delta (last -> current)
  {
    lock_guard<mutex> lock(m_odom_delta_mutex);
    m_odom_delta = m_odom_delta * last_odom_tf_.inverse() * m_current_keyframe.pose_eig;
    Eigen::Matrix4d current_odom_ = m_last_corrected_pose * m_odom_delta;
    m_realtime_pose_pub.publish(pose_eig_to_pose_stamped(current_odom_, m_map_frame));
  }
  return;
}
void FAST_LIO_SAM_CLASS::pgo_timer_func(const ros::TimerEvent& event)
{
  if (!m_init) return;
  high_resolution_clock::time_point t1_ = high_resolution_clock::now();
  //// 1. check if keyframe and save
  bool if_keyframe_ = false;
  {
    lock_guard<mutex> lock(m_pose_pcd_mutex);
    if_keyframe_ = check_if_keyframe(m_current_keyframe);
    if (if_keyframe_) m_keyframes.push_back(m_current_keyframe);
  }
  high_resolution_clock::time_point t2_ = high_resolution_clock::now();
  
  if (if_keyframe_) //if so,
  {
    //vis
    {
      lock_guard<mutex> lock(m_vis_vars_mutex);
      update_vis_vars(m_keyframes.back());
    }

    //// 2. add to graph
    //graph
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished());
    gtsam::Pose3 pose_from_ = m_keyframes[m_keyframes.size()-2].pose_gtsam;
    gtsam::Pose3 pose_to_ = m_keyframes.back().pose_gtsam;
    m_gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(m_keyframes.size()-2, m_keyframes.size()-1, pose_from_.between(pose_to_), odom_noise_));
    m_init_esti.insert(m_keyframes.size()-1, pose_to_);
    high_resolution_clock::time_point t3_ = high_resolution_clock::now();
    
    //// 3. check loop-closure and add to graph
    // from current new keyframe to old keyframes in threshold radius,
    for (int idx = 0; idx < m_keyframes.size()-1; ++idx)
    {
      //check if potential loop: close enough in distance, far enough in time
      if (m_loop_det_radi > (m_keyframes[idx].pose_eig.block<3, 1>(0, 3) - m_keyframes.back().pose_eig.block<3, 1>(0, 3)).norm() 
          && m_loop_det_tdiff_thr < (m_keyframes.back().timestamp - m_keyframes[idx].timestamp))
      {
        // match with GICP
        pcl::PointCloud<pcl::PointXYZI>::Ptr src_raw_(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr src_(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr dst_raw_(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr dst_(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> dummy_;
        *src_raw_ = m_keyframes.back().pcd;
        *dst_raw_ = m_keyframes[idx].pcd;
        m_voxelgrid.setInputCloud(src_raw_);
        m_voxelgrid.filter(*src_);
        m_voxelgrid.setInputCloud(dst_raw_);
        m_voxelgrid.filter(*dst_);
        m_gicp.setInputSource(src_);
        m_gicp.setInputTarget(dst_);
        m_gicp.align(dummy_);
        // if matchness score is lower than threshold, (lower is better)
        double score_ = m_gicp.getFitnessScore();
        cout << score_ << endl;
        if(m_gicp.hasConverged() && score_ < m_gicp_score_thr)
        {
          Eigen::Matrix4d pose_between_eig_ = m_gicp.getFinalTransformation().cast<double>();
          gtsam::noiseModel::Diagonal::shared_ptr loop_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << score_, score_, score_, score_, score_, score_).finished());
          m_gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(m_keyframes.size()-1, idx, pose_eig_to_gtsam_pose(pose_between_eig_), loop_noise_));
          m_loop_idx_pairs.push_back({m_keyframes.size()-1, idx}); //for vis
          m_loop_added_flag = true;
        }
      }
    }
    high_resolution_clock::time_point t4_ = high_resolution_clock::now();
    //// 4. optimize with graph
    // m_corrected_esti = gtsam::LevenbergMarquardtOptimizer(m_gtsam_graph, m_init_esti).optimize(); // cf. isam.update vs values.LM.optimize
    m_isam_handler->update(m_gtsam_graph, m_init_esti);
    m_isam_handler->update();
    m_isam_handler->update();
    m_isam_handler->update();
    m_isam_handler->update();
    m_gtsam_graph.resize(0);
    m_init_esti.clear();
    high_resolution_clock::time_point t5_ = high_resolution_clock::now();
    //// 5. processing with corrected poses
    // get corrected results
    {
      lock_guard<mutex> lock(m_corrected_esti_mutex);
      m_corrected_esti = m_isam_handler->calculateEstimate();
      m_last_corrected_pose = gtsam_pose_to_pose_eig(m_corrected_esti.at<gtsam::Pose3>(m_corrected_esti.size()-1));
    }
    // reset odom delta (for realtime pose pub)
    {
      lock_guard<mutex> lock(m_odom_delta_mutex);
      m_odom_delta = Eigen::Matrix4d::Identity();
    }
    // stack pcd in corrected pose frame
    pcl::PointCloud<pcl::PointXYZI> tmp_pcd_;
    pcl::transformPointCloud(m_keyframes.back().pcd, tmp_pcd_, (m_keyframes.back().pose_eig.inverse()*m_last_corrected_pose).cast<float>());
    m_corrected_current_pcd_pub.publish(pcl_to_pcl_ros(tmp_pcd_, m_map_frame));
    {
      lock_guard<mutex> lock(m_pcd_map_mutex);
      m_corrected_map += tmp_pcd_;
    }
    //if loop closed,
    if (m_loop_added_flag) 
    {
      {
        lock_guard<mutex> lock(m_vis_vars_mutex);
        // correct pose and path
        m_corrected_odoms.clear();
        m_corrected_path.poses.clear();
        for (int i = 0; i < m_corrected_esti.size(); ++i)
        {
          gtsam::Pose3 pose_ = m_corrected_esti.at<gtsam::Pose3>(i);
          m_corrected_odoms.points.emplace_back(pose_.translation().x(), pose_.translation().y(), pose_.translation().z());
          m_corrected_path.poses.push_back(gtsam_pose_to_pose_stamped(pose_, m_map_frame));
        }
      }
      // correct pcd
      {
        lock_guard<mutex> lock(m_pcd_map_mutex);
        m_corrected_map.clear();
        for (int i = 0; i < m_corrected_esti.size(); ++i)
        {
          pcl::PointCloud<pcl::PointXYZI> tmp_pcd_;
          pcl::transformPointCloud(m_keyframes[i].pcd, tmp_pcd_, (m_keyframes[i].pose_eig.inverse()*gtsam_pose_to_pose_eig(m_corrected_esti.at<gtsam::Pose3>(i))).cast<float>());
          m_corrected_map += tmp_pcd_;
        }
      }
      m_loop_added_flag = false;
    }
    high_resolution_clock::time_point t6_ = high_resolution_clock::now();
    ROS_INFO("key: %.1f, graph: %.1f, loop: %.1f, opt: %.1f, corr: %.1f, tot: %.1fms", 
            duration_cast<microseconds>(t2_-t1_).count()/1e3, duration_cast<microseconds>(t3_-t2_).count()/1e3,
            duration_cast<microseconds>(t4_-t3_).count()/1e3, duration_cast<microseconds>(t5_-t4_).count()/1e3,
            duration_cast<microseconds>(t6_-t5_).count()/1e3, duration_cast<microseconds>(t6_-t1_).count()/1e3);
  }
  //// 6. visualize
  high_resolution_clock::time_point tv1_ = high_resolution_clock::now();
  {
    lock_guard<mutex> lock(m_corrected_esti_mutex);
    if (!m_loop_idx_pairs.empty()) m_loop_detection_pub.publish(get_loop_markers());
  }
  m_odom_pub.publish(pcl_to_pcl_ros(m_odoms, m_map_frame));
  m_path_pub.publish(m_odom_path);
  {
    lock_guard<mutex> lock(m_vis_vars_mutex);
    m_corrected_odom_pub.publish(pcl_to_pcl_ros(m_corrected_odoms, m_map_frame));
    m_corrected_path_pub.publish(m_corrected_path);
  }
  {
    lock_guard<mutex> lock(m_pcd_map_mutex);
    m_corrected_pcd_map_pub.publish(pcl_to_pcl_ros(m_corrected_map, m_map_frame));
  }
  high_resolution_clock::time_point tv2_ = high_resolution_clock::now();
  ROS_INFO("vis: %.1fms", duration_cast<microseconds>(tv2_-tv1_).count()/1e3);
  return;
}


void FAST_LIO_SAM_CLASS::update_vis_vars(const pose_pcd &pose_pcd_in)
{
  m_odoms.points.emplace_back(pose_pcd_in.pose_eig(0, 3), pose_pcd_in.pose_eig(1, 3), pose_pcd_in.pose_eig(2, 3));
  m_corrected_odoms.points.emplace_back(pose_pcd_in.pose_eig(0, 3), pose_pcd_in.pose_eig(1, 3), pose_pcd_in.pose_eig(2, 3));
  geometry_msgs::PoseStamped pose_path_ = gtsam_pose_to_pose_stamped(pose_pcd_in.pose_gtsam, m_map_frame);
  m_odom_path.poses.push_back(pose_path_);
  m_corrected_path.poses.push_back(pose_path_);
  return;
}
inline bool FAST_LIO_SAM_CLASS::check_if_keyframe(const pose_pcd &pose_pcd_in)
{
  return m_keyframe_thr < (m_keyframes.back().pose_eig.block<3, 1>(0, 3) - pose_pcd_in.pose_eig.block<3, 1>(0, 3)).norm();
}
visualization_msgs::Marker FAST_LIO_SAM_CLASS::get_loop_markers()
{
  visualization_msgs::Marker edges_; edges_.type = 5u;
  edges_.scale.x = 0.12f; edges_.header.frame_id = m_map_frame; edges_.pose.orientation.w = 1.0f;
  edges_.color.r = 1.0f; edges_.color.g = 1.0f; edges_.color.b = 1.0f; edges_.color.a = 1.0f;
  for (int i = 0; i < m_loop_idx_pairs.size(); ++i)
  {
    gtsam::Pose3 pose_ = m_corrected_esti.at<gtsam::Pose3>(m_loop_idx_pairs[i].first);
    gtsam::Pose3 pose2_ = m_corrected_esti.at<gtsam::Pose3>(m_loop_idx_pairs[i].second);
    geometry_msgs::Point p_, p2_;
    p_.x = pose_.translation().x(); p_.y = pose_.translation().y(); p_.z = pose_.translation().z();
    p2_.x = pose2_.translation().x(); p2_.y = pose2_.translation().y(); p2_.z = pose2_.translation().z();
    edges_.points.push_back(p_);
    edges_.points.push_back(p2_);
  }
  return edges_;
}



#endif