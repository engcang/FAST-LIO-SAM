#include "main.h"


void FAST_LIO_SAM_CLASS::update_vis_vars(const pose_pcd &pose_pcd_in)
{
  m_odoms.points.emplace_back(pose_pcd_in.pose_eig(0, 3), pose_pcd_in.pose_eig(1, 3), pose_pcd_in.pose_eig(2, 3));
  m_corrected_odoms.points.emplace_back(pose_pcd_in.pose_eig(0, 3), pose_pcd_in.pose_eig(1, 3), pose_pcd_in.pose_eig(2, 3));
  geometry_msgs::PoseStamped pose_path_ = gtsam_pose_to_pose_stamped(pose_pcd_in.pose_gtsam, m_map_frame);
  m_odom_path.poses.push_back(pose_path_);
  m_corrected_path.poses.push_back(pose_path_);
  return;
}
bool FAST_LIO_SAM_CLASS::check_if_keyframe(const pose_pcd &pose_pcd_in)
{
  return m_keyframe_thr < (m_keyframes.back().pose_eig.block<3, 1>(0, 3) - pose_pcd_in.pose_eig.block<3, 1>(0, 3)).norm();
}
int FAST_LIO_SAM_CLASS::get_closest_keyframe_idx(const vector<pose_pcd> &keyframes)
{
  double shortest_distance_ = m_loop_det_radi*3.0;
  int closest_idx_ = -1;
  for (int idx = 0; idx < keyframes.size()-1; ++idx)
  {
    //check if potential loop: close enough in distance, far enough in time
    double tmp_dist_ = (keyframes[idx].pose_eig.block<3, 1>(0, 3) - keyframes.back().pose_eig.block<3, 1>(0, 3)).norm();
    if (m_loop_det_radi > tmp_dist_ && m_loop_det_tdiff_thr < (keyframes.back().timestamp - keyframes[idx].timestamp))
    {
      if (tmp_dist_ < shortest_distance_)
      {
        shortest_distance_ = tmp_dist_;
        closest_idx_ = idx;
      }
    }
  }
  return closest_idx_;
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
void FAST_LIO_SAM_CLASS::gicp_key_to_subkeys(const int &closest_idx)
{
	// merge subkeyframes before GICP
  pcl::PointCloud<pcl::PointXYZI>::Ptr dst_raw_(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i = closest_idx-m_sub_key_num; i < closest_idx+m_sub_key_num+1; ++i)
  {
    if (i>=0 && i < m_keyframes.size()-1) //if exists
    {
      *dst_raw_ = *dst_raw_ + m_keyframes[i].pcd;
    }
  }
  // then match with GICP
  pcl::PointCloud<pcl::PointXYZI>::Ptr src_raw_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr src_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dst_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> dummy_;
  *src_raw_ = m_keyframes.back().pcd;
  m_voxelgrid.setInputCloud(src_raw_);
  m_voxelgrid.filter(*src_);
  m_voxelgrid.setInputCloud(dst_raw_);
  m_voxelgrid.filter(*dst_);
  m_gicp.setInputSource(src_);
  m_gicp.setInputTarget(dst_);
  m_gicp.align(dummy_);	
	return;
}