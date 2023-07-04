#ifndef FAST_LIO_SAM_MAIN_H
#define FAST_LIO_SAM_MAIN_H

///// common headers
#include <time.h>
#include <math.h>
#include <cmath>
#include <chrono>
#include <vector>
#include <mutex>
#include <string>
#include <utility> // pair, make_pair
///// ROS
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler
#include <tf_conversions/tf_eigen.h> // tf <-> eigen
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
///// PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <pcl/common/transforms.h> //transformPointCloud
#include <pcl/conversions.h> //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
///// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>


using namespace std;
using namespace std::chrono;
// using namespace Eigen;
// using namespace gtsam; //TODO
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> odom_pcd_sync_pol;


////////////////////////////////////////////////////////////////////////////////////////////////////
struct pose_pcd
{
  pcl::PointCloud<pcl::PointXYZI> pcd;
  gtsam::Pose3 pose_gtsam;
  Eigen::Matrix4d pose_eig;
  bool keyframe=false;
  pose_pcd(const nav_msgs::Odometry &odom_in, const sensor_msgs::PointCloud2 &pcd_in)
  {
    pcl::fromROSMsg(pcd_in, pcd);
    tf::Quaternion q_(odom_in.pose.pose.orientation.x, odom_in.pose.pose.orientation.y, odom_in.pose.pose.orientation.z, odom_in.pose.pose.orientation.w);
    tf::Matrix3x3 m_(q_);
    Eigen::Matrix3d tmp_rot_mat;
    tf::matrixTFToEigen(m_, tmp_rot_mat);
    pose_eig.block<3, 3>(0, 0) = tmp_rot_mat;
    pose_eig(0, 3) = odom_in.pose.pose.position.x;
    pose_eig(1, 3) = odom_in.pose.pose.position.y;
    pose_eig(2, 3) = odom_in.pose.pose.position.z;
    pose_gtsam = gtsam::Pose3(gtsam::Rot3::Quaternion(odom_in.pose.pose.orientation.w, odom_in.pose.pose.orientation.x, odom_in.pose.pose.orientation.y, odom_in.pose.pose.orientation.z), 
                              gtsam::Point3(odom_in.pose.pose.position.x, odom_in.pose.pose.position.y, odom_in.pose.pose.position.z));
  }
};
////////////////////////////////////////////////////////////////////////////////////////////////////
class FAST_LIO_SAM_CLASS
{
  private:
    ///// basic params
    string m_map_frame;
    ///// shared data - odom and pcd
    mutex m_pose_pcd_mutex;
    vector<pose_pcd> m_pose_pcd_vec;
    bool m_init=false;
    int m_last_key_idx=0;
    ///// graph and values
    gtsam::NonlinearFactorGraph m_gtsam_graph;
    gtsam::Values m_init_esti;
    gtsam::Values m_corrected_esti;
    int m_graph_idx=0;
    
    // shared_ptr<Frontier_handler> m_fron_handler = nullptr;
    ///// ros
    ros::NodeHandle m_nh;
    ros::Subscriber m_odom_sub;
    ros::Publisher m_corrected_odom_pub, m_corrected_path_pub, m_corrected_pcd_pub, m_loop_closure_pub;
    // ros::Timer m_graph_timer;
    // odom, pcd sync subscriber
    message_filters::Synchronizer<odom_pcd_sync_pol> *m_sub_odom_pcd_sync;
    message_filters::Subscriber<nav_msgs::Odometry> *m_sub_odom;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *m_sub_pcd;

    ///// functions
  public:
    FAST_LIO_SAM_CLASS(const ros::NodeHandle& n_private);
  private:
    inline bool check_if_keyframe();
    void odom_pcd_cb(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg);
    void graph_timer_func(const ros::TimerEvent& event);
    
};




///////////////////////////////////// can be separated into .cpp files
FAST_LIO_SAM_CLASS::FAST_LIO_SAM_CLASS(const ros::NodeHandle& n_private) : m_nh(n_private)
{
  ////////// ROS params
  // temp vars
  // double graph_update_hz_;
  // m_nh.param<double>("/graph_update_hz", graph_update_hz_, false);
  // m_nh.param<bool>("/computing_time_debug", m_computing_time_debug, false);
  // m_nh.getParam("/cam_tf", cam_tf_);
  // m_fron_handler = make_shared<Frontier_handler>(fron_vox_res_);

  ////////// ROS things
  //// publishers
  // m_kdtree_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/kdtree", 3, true);
  /// subscribers
  // odom, pcd sync subscriber
  m_sub_odom = new message_filters::Subscriber<nav_msgs::Odometry>(m_nh, "/Odometry", 10);
  m_sub_pcd = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh, "/cloud_registered", 10);
  m_sub_odom_pcd_sync = new message_filters::Synchronizer<odom_pcd_sync_pol>(odom_pcd_sync_pol(10), *m_sub_odom, *m_sub_pcd);
  m_sub_odom_pcd_sync->registerCallback(boost::bind(&FAST_LIO_SAM_CLASS::odom_pcd_cb, this, _1, _2));
  //// Timers at the end
  // m_graph_timer = m_nh.createTimer(ros::Duration(1/graph_update_hz_), &FAST_LIO_SAM_CLASS::graph_timer_func, this);
  
  ROS_WARN("Main class, starting node...");
}

void FAST_LIO_SAM_CLASS::odom_pcd_cb(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg)
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  //// 1. save odom/pcd sub
  // {
    // lock_guard<mutex> lock(m_pose_pcd_mutex);
    m_pose_pcd_vec.push_back(pose_pcd(*odom_msg, *pcd_msg));
  // }
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  //// 2. graph update
  if (!m_init) //// 2-0. only once
  {
    //graph
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4).finished()); // rad*rad, meter*meter
    m_gtsam_graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, m_pose_pcd_vec.back().pose_gtsam, prior_noise));
    m_init_esti.insert(0, m_pose_pcd_vec.back().pose_gtsam);
    //others
    m_pose_pcd_vec.back().keyframe = true;
    m_last_key_idx = 0;
    m_init = true;
    return;
  }
  else
  {
    //// 2-1. check if keyframe
    bool if_keyframe = check_if_keyframe(); // TODO: adaptively check keyframe
    high_resolution_clock::time_point t3 = high_resolution_clock::now();
    
    if (if_keyframe)
    {
      //// 2-2. if so, add to graph
      //graph
      gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished()); //TODO: adaptively parameterizing
      gtsam::Pose3 pose_from = m_pose_pcd_vec[m_last_key_idx].pose_gtsam;
      gtsam::Pose3 pose_to = m_pose_pcd_vec.back().pose_gtsam;
      m_gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(m_graph_idx, m_graph_idx+1, pose_from.between(pose_to), odom_noise));
      m_init_esti.insert(m_graph_idx+1, pose_to);
      m_graph_idx++;
      //others
      m_pose_pcd_vec.back().keyframe = true;
      m_last_key_idx = m_pose_pcd_vec.size()-1;
      high_resolution_clock::time_point t4 = high_resolution_clock::now();
      //// 2-3. if so, check loop-closure and add to graph
      // from current new keyframe to old keyframes in threshold radius,
      // if matchness score is higher than threshold,
      high_resolution_clock::time_point t5 = high_resolution_clock::now();
      //// 3. optimize with graph
      m_corrected_esti = gtsam::LevenbergMarquardtOptimizer(m_gtsam_graph, m_init_esti).optimize();
      high_resolution_clock::time_point t6 = high_resolution_clock::now();
      //// 4. correct poses in path
      high_resolution_clock::time_point t7 = high_resolution_clock::now();
      //// 5. correct pcd
      high_resolution_clock::time_point t8 = high_resolution_clock::now();
      //// 6. visualize
      high_resolution_clock::time_point t9 = high_resolution_clock::now();
      
      ROS_INFO("add: %.1f, key: %.1f, graph: %.1f, loop: %.1f, opt: %.1f, cor_p: %.1f, cor_l: %.1f, vis: %.1f, tot: %.1fms", 
              duration_cast<microseconds>(t2-t1).count()/1e3, duration_cast<microseconds>(t3-t2).count()/1e3,
              duration_cast<microseconds>(t4-t3).count()/1e3, duration_cast<microseconds>(t5-t4).count()/1e3,
              duration_cast<microseconds>(t6-t5).count()/1e3, duration_cast<microseconds>(t7-t6).count()/1e3,
              duration_cast<microseconds>(t8-t7).count()/1e3, duration_cast<microseconds>(t9-t8).count()/1e3,
              duration_cast<microseconds>(t9-t1).count()/1e3);
    }
  }

  // gtsam::Pose3(gtsam::Rot3::RzRyRx(r, p, y), gtsam::Point3(x, y, z));
  // transformTobeMapped[0] = latestEstimate.rotation().roll();
  // transformTobeMapped[1] = latestEstimate.rotation().pitch();
  // transformTobeMapped[2] = latestEstimate.rotation().yaw();
  // transformTobeMapped[3] = latestEstimate.translation().x();
  // transformTobeMapped[4] = latestEstimate.translation().y();
  // transformTobeMapped[5] = latestEstimate.translation().z();

  return;
}

inline bool FAST_LIO_SAM_CLASS::check_if_keyframe()
{
  return 1.0 < (m_pose_pcd_vec[m_last_key_idx].pose_eig.block<3, 1>(0, 3) - m_pose_pcd_vec.back().pose_eig.block<3, 1>(0, 3)).norm();
}




#endif