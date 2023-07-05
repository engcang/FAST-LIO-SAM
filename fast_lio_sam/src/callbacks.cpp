#include "main.h"


void FAST_LIO_SAM_CLASS::odom_pcd_cb(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg)
{
  Eigen::Matrix4d last_odom_tf_ = m_current_keyframe.pose_eig; //to calculate delta
  {
    lock_guard<mutex> lock(m_pose_pcd_mutex);
    m_current_keyframe = pose_pcd(*odom_msg, *pcd_msg); //save current odom to check if keyframe or not
  }
  if (!m_init) //// init only once
  {
    //others
    m_keyframes.push_back(m_current_keyframe);
    m_corrected_map += m_keyframes.back().pcd;
    update_vis_vars(m_keyframes.back());
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
    update_vis_vars(m_keyframes.back());

    //// 2. add to graph
    //graph
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished());
    gtsam::Pose3 pose_from_ = m_keyframes[m_keyframes.size()-2].pose_gtsam;
    gtsam::Pose3 pose_to_ = m_keyframes.back().pose_gtsam;
    m_gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(m_keyframes.size()-2, m_keyframes.size()-1, pose_from_.between(pose_to_), odom_noise_));
    m_init_esti.insert(m_keyframes.size()-1, pose_to_);
    high_resolution_clock::time_point t3_ = high_resolution_clock::now();
    
    //// 3. check loop-closure and add to graph
    // from current new keyframe to old keyframes in threshold radius, get the closest keyframe
    int closest_keyframe_idx_ = get_closest_keyframe_idx(m_keyframes);
    if (closest_keyframe_idx_ >= 0) //if exists
    {
    	// GICP to check loop
    	gicp_key_to_subkeys(closest_keyframe_idx_);
      double score_ = m_gicp.getFitnessScore();
      cout << score_ << endl;
      // if matchness score is lower than threshold, (lower is better)
      if(m_gicp.hasConverged() && score_ < m_gicp_score_thr) // add loop factor
      {
        Eigen::Matrix4d pose_between_eig_ = m_gicp.getFinalTransformation().cast<double>();
        gtsam::noiseModel::Diagonal::shared_ptr loop_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << score_, score_, score_, score_, score_, score_).finished());
        m_gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(m_keyframes.size()-1, closest_keyframe_idx_, pose_eig_to_gtsam_pose(pose_between_eig_), loop_noise_));
        m_loop_idx_pairs.push_back({m_keyframes.size()-1, closest_keyframe_idx_}); //for vis
        m_loop_added_flag = true;
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
    m_corrected_esti = m_isam_handler->calculateEstimate();
    m_last_corrected_pose = gtsam_pose_to_pose_eig(m_corrected_esti.at<gtsam::Pose3>(m_corrected_esti.size()-1));
    // reset odom delta (for realtime pose pub)
    {
      lock_guard<mutex> lock(m_odom_delta_mutex);
      m_odom_delta = Eigen::Matrix4d::Identity();
    }
    // stack pcd in corrected pose frame
    pcl::PointCloud<pcl::PointXYZI> tmp_pcd_;
    pcl::transformPointCloud(m_keyframes.back().pcd, tmp_pcd_, (m_keyframes.back().pose_eig.inverse()*m_last_corrected_pose).cast<float>());
    m_corrected_map += tmp_pcd_;
    m_corrected_current_pcd_pub.publish(pcl_to_pcl_ros(tmp_pcd_, m_map_frame)); // pub current scan in corrected pose frame
    //if loop closed,
    if (m_loop_added_flag) 
    {
      // correct pose and path
      m_corrected_odoms.clear();
      m_corrected_path.poses.clear();
      for (int i = 0; i < m_corrected_esti.size(); ++i)
      {
        gtsam::Pose3 pose_ = m_corrected_esti.at<gtsam::Pose3>(i);
        m_corrected_odoms.points.emplace_back(pose_.translation().x(), pose_.translation().y(), pose_.translation().z());
        m_corrected_path.poses.push_back(gtsam_pose_to_pose_stamped(pose_, m_map_frame));
      }
      // correct pcd
      m_corrected_map.clear();
      for (int i = 0; i < m_corrected_esti.size(); ++i)
      {
        pcl::PointCloud<pcl::PointXYZI> tmp_pcd_;
        pcl::transformPointCloud(m_keyframes[i].pcd, tmp_pcd_, (m_keyframes[i].pose_eig.inverse()*gtsam_pose_to_pose_eig(m_corrected_esti.at<gtsam::Pose3>(i))).cast<float>());
        m_corrected_map += tmp_pcd_;
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
  if (!m_loop_idx_pairs.empty()) m_loop_detection_pub.publish(get_loop_markers());
  m_odom_pub.publish(pcl_to_pcl_ros(m_odoms, m_map_frame));
  m_path_pub.publish(m_odom_path);
  m_corrected_odom_pub.publish(pcl_to_pcl_ros(m_corrected_odoms, m_map_frame));
  m_corrected_path_pub.publish(m_corrected_path);
  m_corrected_pcd_map_pub.publish(pcl_to_pcl_ros(m_corrected_map, m_map_frame));
  high_resolution_clock::time_point tv2_ = high_resolution_clock::now();
  ROS_INFO("vis: %.1fms", duration_cast<microseconds>(tv2_-tv1_).count()/1e3);
  return;
}