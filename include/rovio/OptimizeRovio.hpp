#ifndef ROVIO_OPTIMIZEROVIO_HPP_
#define ROVIO_OPTIMIZEROVIO_HPP_

#include <ros/package.h>
#include <memory>
#include <limits>
#include "rovio/RovioFilter.hpp"
#include "rovio/RovioNode.hpp"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nlopt.h>
#include <tf/transform_broadcaster.h>

#ifdef ROVIO_NMAXFEATURE
static constexpr int nMax_ = ROVIO_NMAXFEATURE;
#else
static constexpr int nMax_ = 25; // Maximal number of considered features in the filter state.
#endif

#ifdef ROVIO_NLEVELS
static constexpr int nLevels_ = ROVIO_NLEVELS;
#else
static constexpr int nLevels_ = 4; // // Total number of pyramid levels considered.
#endif

#ifdef ROVIO_PATCHSIZE
static constexpr int patchSize_ = ROVIO_PATCHSIZE;
#else
static constexpr int patchSize_ = 6; // Edge length of the patches (in pixel). Must be a multiple of 2!
#endif

#ifdef ROVIO_NCAM
static constexpr int nCam_ = ROVIO_NCAM;
#else
static constexpr int nCam_ = 2; // Used total number of cameras.
#endif

#ifdef ROVIO_NPOSE
static constexpr int nPose_ = ROVIO_NPOSE;
#else
static constexpr int nPose_ = 0; // Additional pose states.
#endif

typedef rovio::RovioFilter<rovio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;

// NLOPT Objective function
double testNloptFunc(unsigned n, const double* x, double* grad, void* my_func_data);

class RovioOptimizer
{
 public:
  RovioOptimizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& bagFileName, const std::string& groundTruthTopic,
                 const std::string& rovioCam0Topic, const std::string& rovioCam1Topic, const std::string& rovioImuTopic);
  ~RovioOptimizer() {};

  // ROVIO optimization routine using NLOPT
  void optimizeRovioUsingNlopt();

  // Evaluate performance and return performance measure
  double evaluationCriterion();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  rosbag::Bag bag_;
  rosbag::View bag_view_;
  rosbag::View::iterator current_bag_pos_;
  int num_associations;

  std::string ground_truth_topic_, ground_truth_odometry_topic_;
  std::string rovio_cam0_topic_, rovio_cam1_topic_, rovio_imu_topic_;

  sensor_msgs::ImageConstPtr rovio_cam0_msg_, rovio_cam1_msg_;
  nav_msgs::Odometry::ConstPtr odom_msg_, ground_truth_odometry_msg_;
  sensor_msgs::Imu::ConstPtr imu_msg_, rovio_imu_msg_;
  geometry_msgs::TransformStamped::ConstPtr ground_truth_msg_;

  ros::Time latest_groundtruth_time_, latest_estimate_time_;
  V3D initial_position_, final_position_;

  // debug
  tf::TransformBroadcaster br_;
  tf::Transform tf_t_;

  // Node
  rovio::RovioNode<mtFilter> *rovioNode_;
  std::string filter_config, optimized_filter_config;

  // Filter
  std::shared_ptr<mtFilter> rovioFilter_;
};

#endif // ROVIO_OPTIMIZEROVIO_HPP_
