#ifndef FAST_LIO_SAM_LOOP_CLOSURE_H
#define FAST_LIO_SAM_LOOP_CLOSURE_H

///// C++ common headers
#include <tuple>
#include <vector>
#include <memory>
#include <limits>
#include <iostream>
///// PCL
#include <pcl/point_types.h>      //pt
#include <pcl/point_cloud.h>      //cloud
#include <pcl/registration/icp.h> //icp
///// Eigen
#include <Eigen/Eigen>
///// coded headers
#include "pose_pcd.hpp"
#include "utilities.hpp"
using PcdPair = std::tuple<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>;

struct LoopClosureConfig
{
    int num_submap_keyframes_ = 10;
    double voxel_res_ = 0.1;
    double loop_detection_radius_;
    double loop_detection_timediff_threshold_;
    double icp_score_threshold_;
    double icp_max_corr_dist_;
};

struct RegistrationOutput
{
    bool is_valid_ = false;
    bool is_converged_ = false;
    double score_ = std::numeric_limits<double>::max();
    Eigen::Matrix4d pose_between_eig_ = Eigen::Matrix4d::Identity();
};

class LoopClosure
{
private:
    pcl::IterativeClosestPoint<PointType, PointType> icp_handler_;
    int closest_keyframe_idx_ = -1;
    pcl::PointCloud<PointType>::Ptr src_cloud_;
    pcl::PointCloud<PointType>::Ptr dst_cloud_;
    pcl::PointCloud<PointType> aligned_;
    LoopClosureConfig config_;

public:
    explicit LoopClosure(const LoopClosureConfig &config);
    ~LoopClosure();
    int fetchClosestKeyframeIdx(const PosePcd &query_keyframe,
                                const std::vector<PosePcd> &keyframes);
    PcdPair setSrcAndDstCloud(const std::vector<PosePcd> &keyframes,
                              const int src_idx,
                              const int dst_idx,
                              const int submap_range,
                              const double voxel_res);
    RegistrationOutput icpAlignment(const pcl::PointCloud<PointType> &src,
                                    const pcl::PointCloud<PointType> &dst);
    RegistrationOutput performLoopClosure(const PosePcd &query_keyframe,
                                          const std::vector<PosePcd> &keyframes);
    RegistrationOutput performLoopClosure(const PosePcd &query_keyframe,
                                          const std::vector<PosePcd> &keyframes,
                                          const int closest_keyframe_idx);
    pcl::PointCloud<PointType> getSourceCloud();
    pcl::PointCloud<PointType> getTargetCloud();
    pcl::PointCloud<PointType> getFinalAlignedCloud();
    int getClosestKeyframeidx();
};

#endif
