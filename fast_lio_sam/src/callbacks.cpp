#include "main.h"


void FAST_LIO_SAM_CLASS::odom_pcd_cb(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg)
{
  Eigen::Matrix4d last_odom_tf_, current_odom_;
  last_odom_tf_ = m_current_frame.pose_eig; //to calculate delta
  m_current_frame = pose_pcd(*odom_msg, *pcd_msg, m_current_keyframe_idx); //to be checked if keyframe or not

  if (!m_init) //// init only once
  {
    //others
    m_keyframes.push_back(m_current_frame);
    m_corrected_map = m_current_frame.pcd;
    update_vis_vars(m_current_frame);
    m_corrected_current_pcd_pub.publish(pcl_to_pcl_ros(m_current_frame.pcd, m_map_frame));
    //graph
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4).finished()); // rad*rad, meter*meter
    m_gtsam_graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, m_current_frame.pose_gtsam, prior_noise_));
    m_init_esti.insert(m_current_keyframe_idx, m_current_frame.pose_gtsam);
    m_current_keyframe_idx++;
    m_init = true;
  }
  else
  {
    //// 1. realtime pose = last corrected odom * delta (last -> current)
    high_resolution_clock::time_point t1_ = high_resolution_clock::now();
    {
      lock_guard<mutex> lock(m_realtime_pose_mutex);
      m_odom_delta = m_odom_delta * last_odom_tf_.inverse() * m_current_frame.pose_eig;      
      current_odom_ = m_last_corrected_pose * m_odom_delta;
      m_realtime_pose_pub.publish(pose_eig_to_pose_stamped(current_odom_, m_map_frame));
    }

    //// 2. check if keyframe
    high_resolution_clock::time_point t2_ = high_resolution_clock::now();
    if (check_if_keyframe(m_current_frame, m_keyframes.back()))
    {
      // 2-2. if so, save
      {
        lock_guard<mutex> lock(m_keyframes_mutex);
        m_keyframes.push_back(m_current_frame);
        m_not_processed_keyframes.push_back(m_current_frame); //to check loop in another thread
      }
      // 2-3. if so, add to graph
      gtsam::noiseModel::Diagonal::shared_ptr odom_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished());
      gtsam::Pose3 pose_from_ = m_keyframes[m_current_keyframe_idx-1].pose_gtsam;
      gtsam::Pose3 pose_to_ = m_current_frame.pose_gtsam;
      {
        lock_guard<mutex> lock(m_graph_mutex);
        m_gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(m_current_keyframe_idx-1, m_current_keyframe_idx, pose_from_.between(pose_to_), odom_noise_));
        m_init_esti.insert(m_current_keyframe_idx, pose_to_);
      }
      m_current_keyframe_idx++;
      high_resolution_clock::time_point t3_ = high_resolution_clock::now();

      //// 3. vis
      pcl::PointCloud<pcl::PointXYZI> tmp_pcd_;
      pcl::transformPointCloud(m_current_frame.pcd, tmp_pcd_, (m_current_frame.pose_eig.inverse()*current_odom_).cast<float>());
      {
        lock_guard<mutex> lock(m_vis_mutex);
        m_corrected_map += tmp_pcd_; // stack pcd in corrected pose frame
        update_vis_vars(m_current_frame);
      }
      m_corrected_current_pcd_pub.publish(pcl_to_pcl_ros(tmp_pcd_, m_map_frame)); // pub current scan in corrected pose frame
      high_resolution_clock::time_point t4_ = high_resolution_clock::now();

      ROS_INFO("real_pub: %.1f, keycheck_add: %.1fms, vis: %.1fms", 
        duration_cast<microseconds>(t2_-t1_).count()/1e3, duration_cast<microseconds>(t3_-t2_).count()/1e3,
        duration_cast<microseconds>(t4_-t3_).count()/1e3);
    }
  }
  return;
}

void FAST_LIO_SAM_CLASS::pgo_timer_func(const ros::TimerEvent& event)
{
  if (!m_init) return;
  
  //// 1. copy keyframes and not processed keyframes
  high_resolution_clock::time_point t1_ = high_resolution_clock::now();
  deque<pose_pcd> not_processed_keyframes_copy_;
  vector<pose_pcd> keyframes_copy_;
  if (!m_not_processed_keyframes.empty())
  {
    lock_guard<mutex> lock(m_keyframes_mutex);
    keyframes_copy_ = m_keyframes;
    not_processed_keyframes_copy_ = m_not_processed_keyframes;
    m_not_processed_keyframes.clear();
  }

  //// 2. detect loop and add to graph
  high_resolution_clock::time_point t2_ = high_resolution_clock::now();
  bool if_loop_occured_ = false;
  while (!not_processed_keyframes_copy_.empty())
  {
    pose_pcd front_keyframe_ = not_processed_keyframes_copy_.front();
    not_processed_keyframes_copy_.pop_front();

    // from front_keyframe_ keyframe to old keyframes in threshold radius, get the closest keyframe
    int closest_keyframe_idx_ = get_closest_keyframe_idx(front_keyframe_, keyframes_copy_);
    if (closest_keyframe_idx_ >= 0) //if exists
    {
    	// GICP to check loop (from front_keyframe to closest keyframe's neighbor)
    	gicp_key_to_subkeys(front_keyframe_, closest_keyframe_idx_, keyframes_copy_);
      double score_ = m_gicp.getFitnessScore();
      cout << score_ << endl;
      // if matchness score is lower than threshold, (lower is better)
      if(m_gicp.hasConverged() && score_ < m_gicp_score_thr) // add loop factor
      {
        Eigen::Matrix4d pose_between_eig_ = m_gicp.getFinalTransformation().cast<double>();
        gtsam::noiseModel::Diagonal::shared_ptr loop_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << score_, score_, score_, score_, score_, score_).finished());
        {
          lock_guard<mutex> lock(m_graph_mutex);
          m_gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(front_keyframe_.idx, closest_keyframe_idx_, pose_eig_to_gtsam_pose(pose_between_eig_), loop_noise_));
        }
        m_loop_idx_pairs.push_back({front_keyframe_.idx, closest_keyframe_idx_}); //for vis
        if_loop_occured_ = true;
      }
    }
  }

  //// 3. optimize with graph
  high_resolution_clock::time_point t3_ = high_resolution_clock::now();
  // m_corrected_esti = gtsam::LevenbergMarquardtOptimizer(m_gtsam_graph, m_init_esti).optimize(); // cf. isam.update vs values.LM.optimize
  {
    lock_guard<mutex> lock(m_graph_mutex);
    m_isam_handler->update(m_gtsam_graph, m_init_esti);
    m_gtsam_graph.resize(0);
    m_init_esti.clear();
  }
  m_isam_handler->update();
  m_isam_handler->update();
  m_isam_handler->update();
  m_isam_handler->update();

  //// 4. handle corrected results
  high_resolution_clock::time_point t4_ = high_resolution_clock::now();
  // reset odom delta (for realtime pose pub)
  {
    lock_guard<mutex> lock(m_realtime_pose_mutex);
    m_corrected_esti = m_isam_handler->calculateEstimate();
    m_odom_delta = Eigen::Matrix4d::Identity();
    m_last_corrected_pose = gtsam_pose_to_pose_eig(m_corrected_esti.at<gtsam::Pose3>(m_corrected_esti.size()-1));
  }
  high_resolution_clock::time_point t5_ = high_resolution_clock::now();
  
  ROS_INFO("copy: %.1f, loop: %.1f, opt: %.1f, res: %.1f", 
          duration_cast<microseconds>(t2_-t1_).count()/1e3, duration_cast<microseconds>(t3_-t2_).count()/1e3,
          duration_cast<microseconds>(t4_-t3_).count()/1e3, duration_cast<microseconds>(t5_-t4_).count()/1e3);

  if (if_loop_occured_) m_loop_added_flag = true;

  return;
}

void FAST_LIO_SAM_CLASS::vis_timer_func(const ros::TimerEvent& event)
{
  if (!m_init) return;

  high_resolution_clock::time_point tv1_ = high_resolution_clock::now();
  //if loop closed,
  if (m_loop_added_flag) 
  {
    // copy and ready
    vector<pose_pcd> keyframes_copy_;
    gtsam::Values corrected_esti_copy_;
    pcl::PointCloud<pcl::PointXYZ> corrected_odoms_;
    pcl::PointCloud<pcl::PointXYZI> corrected_map_;
    nav_msgs::Path corrected_path_;
    {
      lock_guard<mutex> lock(m_keyframes_mutex);
      keyframes_copy_ = m_keyframes;
    }
    {
      lock_guard<mutex> lock(m_realtime_pose_mutex);
      corrected_esti_copy_ = m_corrected_esti;
    }
    // correct pose and path
    for (int i = 0; i < corrected_esti_copy_.size(); ++i)
    {
      gtsam::Pose3 pose_ = corrected_esti_copy_.at<gtsam::Pose3>(i);
      corrected_odoms_.points.emplace_back(pose_.translation().x(), pose_.translation().y(), pose_.translation().z());
      corrected_path_.poses.push_back(gtsam_pose_to_pose_stamped(pose_, m_map_frame));
    }
    // correct pcd
    int iter_ = corrected_esti_copy_.size() < keyframes_copy_.size() ? corrected_esti_copy_.size() : keyframes_copy_.size();
    for (int i = 0; i < iter_; ++i)
    {
      pcl::PointCloud<pcl::PointXYZI> tmp_pcd_;
      pcl::transformPointCloud(keyframes_copy_[i].pcd, tmp_pcd_, (keyframes_copy_[i].pose_eig.inverse()*gtsam_pose_to_pose_eig(corrected_esti_copy_.at<gtsam::Pose3>(i))).cast<float>());
      corrected_map_ += tmp_pcd_;
    }
    // update vis of loop constraints
    if (!m_loop_idx_pairs.empty())
    {
      m_loop_detection_pub.publish(get_loop_markers(corrected_esti_copy_));
    }
    // update with corrected data
    {
      lock_guard<mutex> lock(m_vis_mutex);
      m_corrected_odoms = corrected_odoms_;
      m_corrected_path.poses = corrected_path_.poses;
      m_corrected_map = corrected_map_;
    }
    m_loop_added_flag = false;
  }
  // voxlize_pcd(m_corrected_map);
  {
    lock_guard<mutex> lock(m_vis_mutex);
    m_odom_pub.publish(pcl_to_pcl_ros(m_odoms, m_map_frame));
    m_path_pub.publish(m_odom_path);
    m_corrected_odom_pub.publish(pcl_to_pcl_ros(m_corrected_odoms, m_map_frame));
    m_corrected_path_pub.publish(m_corrected_path);
    m_corrected_pcd_map_pub.publish(pcl_to_pcl_ros(m_corrected_map, m_map_frame));
  }
  high_resolution_clock::time_point tv2_ = high_resolution_clock::now();
  ROS_INFO("vis: %.1fms", duration_cast<microseconds>(tv2_-tv1_).count()/1e3);
  return;
}