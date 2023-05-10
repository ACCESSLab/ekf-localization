/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * Modified by ACCESS Lab (NC A&T SU) on May 1st, 2023 to include:
 * - subscription to an additional pose message (w/wo covariance)
 * - subscription to weight messages for the gating process
 * - publish the additional pose message
 * - publish ekf status for rviz visualization
 * - publish pose errors (abse, rmse, and mae) in debug info  
 * - synchronization policy for pose messages
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "visualization_msgs/Marker.h"
#include <jsk_rviz_plugins/OverlayText.h>
#include <tuple>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "amathutils_lib/kalman_filter.hpp"
#include "amathutils_lib/time_delay_kalman_filter.hpp"

class EKFLocalizer
{
public:
  EKFLocalizer();
  ~EKFLocalizer();

private:
  ros::NodeHandle nh_;                   //!< @brief ros public node handler
  ros::NodeHandle pnh_;                  //!< @brief  ros private node handler
  ros::Publisher pub_pose_;              //!< @brief ekf estimated pose publisher
  ros::Publisher pub_pose_cov_;          //!< @brief estimated ekf pose with covariance publisher
  ros::Publisher pub_twist_;             //!< @brief ekf estimated twist publisher
  ros::Publisher pub_twist_cov_;         //!< @brief ekf estimated twist with covariance publisher
  ros::Publisher pub_debug_;            //!< @brief debug info publisher
  ros::Publisher pub_measured_pose1_;    //!< @brief debug measurement pose1 publisher
  ros::Publisher pub_measured_pose2_;    //!< @brief debug measurement pose2 publisher
  ros::Publisher pub_yaw_bias_;          //!< @brief ekf estimated yaw bias publisher
  ros::Publisher pub_overlay_info_text_; //!< @brief debug info publisher
  ros::Publisher pub_ekf_status_str_;    //!< @brief debug info publisher
  ros::Publisher pub_ekf_status_int_;    //!< @brief debug info publisher
  ros::Subscriber sub_initialpose_;      //!< @brief initial pose subscriber
  ros::Subscriber sub_pose1_;            //!< @brief measurement pose1 subscriber
  ros::Subscriber sub_pose2_;            //!< @brief measurement pose2 subscriber
  ros::Subscriber sub_weight1_;          //!< @brief measurement weight1 subscriber
  ros::Subscriber sub_weight2_;          //!< @brief measurement weight2 subscriber
  ros::Subscriber sub_twist_;            //!< @brief measurement twist subscriber
  ros::Subscriber sub_pose1_with_cov_;   //!< @brief measurement pose1 with covariance subscriber
  ros::Subscriber sub_pose2_with_cov_;   //!< @brief measurement pose2 with covariance subscriber
  ros::Subscriber sub_twist_with_cov_;   //!< @brief measurement twist with covariance subscriber
  ros::Timer timer_control_;             //!< @brief time for ekf calculation callback
  tf2_ros::TransformBroadcaster tf_br_;  //!< @brief tf broadcaster

  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose1_sync_; //!< @brief measurement pose1 subscriber
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose2_sync_; //!< @brief measurement pose2 subscriber
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, 
                                                          geometry_msgs::PoseStamped> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  TimeDelayKalmanFilter ekf_;  //!< @brief  extended kalman filter instance.

  /* parameters */
  bool show_debug_info_;
  double ekf_rate_;                  //!< @brief  EKF predict rate
  double ekf_dt_;                    //!< @brief  = 1 / ekf_rate_
  bool enable_yaw_bias_estimation_;  //!< @brief  for LiDAR mount error. if true, publish /estimate_yaw_bias
  std::string pose_frame_id_;
  std::string output_frame_id_;
  int counter_pose1_status_ = 1;
  int counter_pose2_status_ = 1;
  bool use_synchronization_;
  
  std::vector<geometry_msgs::PoseStamped> current_pose1_vec_, current_pose2_vec_;
  struct Error
  {
    double pos;
    double yaw;
  };
  Error abse, rmse, mae {0.0, 0.0};

  int dim_x_;              //!< @brief  dimension of EKF state
  int extend_state_step_;  //!< @brief  for time delay compensation
  int dim_x_ex_;           //!< @brief  dimension of extended EKF state (dim_x_ * extended_state_step)

  /* Pose */
  double pose_additional_delay_;          //!< @brief  compensated pose delay time = (pose.header.stamp - now) +
                                          //!< additional_delay [s]
  double pose1_additional_delay_;
  double pose2_additional_delay_;
  double pose_measure_uncertainty_time_;  //!< @brief  added for measurement covariance
  double pose_rate_;                      //!< @brief  pose rate [s], used for covariance calculation
  double pose_gate_dist_;                 //!< @brief  pose measurement is ignored if the maharanobis distance is larger than this
                                          //!< value.
  double pose1_weight_ = 1.0;             //!< @brief  current measured weight1
  double pose2_weight_ = 1.0;             //!< @brief  current measured weight2
  double pose_stddev_x_;                  //!< @brief  standard deviation for pose position x [m]
  double pose_stddev_y_;                  //!< @brief  standard deviation for pose position y [m]
  double pose_stddev_yaw_;                //!< @brief  standard deviation for pose position yaw [rad]
  bool use_pose_with_covariance_;         //!< @brief  use covariance in pose_with_covarianve message
  
  double pose1_rate_;
  double pose1_gate_dist_;
  double pose1_stddev_x_;
  double pose1_stddev_y_;
  double pose1_stddev_yaw_;

  double pose2_rate_;
  double pose2_gate_dist_;
  double pose2_stddev_x_;
  double pose2_stddev_y_;
  double pose2_stddev_yaw_;

  /* twist */
  double twist_additional_delay_;         //!< @brief  compensated delay time = (twist.header.stamp - now) + additional_delay
                                          //!< [s]
  double twist_rate_;                     //!< @brief  rate [s], used for covariance calculation
  double twist_gate_dist_;                //!< @brief  measurement is ignored if the maharanobis distance is larger than this value.
  double twist_stddev_vx_;                //!< @brief  standard deviation for linear vx
  double twist_stddev_wz_;                //!< @brief  standard deviation for angular wx
  bool use_twist_with_covariance_;        //!< @brief  use covariance in twist_with_covarianve message

  /* process noise variance for discrete model */
  double proc_cov_yaw_d_;       //!< @brief  discrete yaw process noise
  double proc_cov_yaw_bias_d_;  //!< @brief  discrete yaw bias process noise
  double proc_cov_vx_d_;        //!< @brief  discrete process noise in d_vx=0
  double proc_cov_wz_d_;        //!< @brief  discrete process noise in d_wz=0

  enum IDX
  {
    X = 0,
    Y = 1,
    YAW = 2,
    YAWB = 3,
    VX = 4,
    WZ = 5,
  };

  /* for model prediction */
  std::shared_ptr<geometry_msgs::TwistStamped> current_twist_ptr_;  //!< @brief current measured twist
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose1_ptr_;   //!< @brief current measured pose1
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose2_ptr_;   //!< @brief current measured pose2
  geometry_msgs::PoseStamped current_pose1_;                        //!< @brief current measured pose1
  geometry_msgs::PoseStamped current_pose2_;                        //!< @brief current measured pose2
  geometry_msgs::PoseStamped current_ekf_pose_;                     //!< @brief current estimated pose
  geometry_msgs::TwistStamped current_ekf_twist_;                   //!< @brief current estimated twist
  boost::array<double, 36ul> current_pose_covariance_;
  boost::array<double, 36ul> current_twist_covariance_;
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose1_sync_ptr_;  //!< @brief current measured pose1
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose2_sync_ptr_;  //!< @brief current measured pose2
  geometry_msgs::PoseStamped current_pose1_sync_;                       //!< @brief current measured pose1
  geometry_msgs::PoseStamped current_pose2_sync_;                       //!< @brief current measured pose2
  // to use height of pose1 in pose2
  float prev_pose_position_z = 0;
  
  enum MethodType
  {
    LIDAR = 0,
    GNSS = 1,
    LIDAR_GNSS = 2,
    DEAD_RECKONING = 3,
  };
  MethodType method_type_ = LIDAR;
  MethodType ekf_status_ = DEAD_RECKONING;                                           //!< @brief ekf status
  bool current_pose1_flag_ = false;
  bool current_pose2_flag_ = false;
  bool mahalanobis_gate_flag_ = false;
  
  const std::vector<std::string> ekf_status_names = {"LIDAR", "GNSS", "LIDAR_GNSS", "DEAD_RECKONING"};
  jsk_rviz_plugins::OverlayText ekf_lidar_text_;
  jsk_rviz_plugins::OverlayText ekf_gnss_text_;
  jsk_rviz_plugins::OverlayText ekf_lidar_gnss_text_;
  jsk_rviz_plugins::OverlayText ekf_dead_reckoning_text_;
  
  /**
   * @brief computes update & prediction of EKF for each ekf_dt_[s] time
   */
  void timerCallback(const ros::TimerEvent& e);

  /**
   * @brief publish tf
   */
  void broadcastTF();

  /**
   * @brief set weight measurement
   */
  void callbackWeight1(const std_msgs::Float64MultiArray::ConstPtr& msg);

  /**
   * @brief set weight measurement
   */
  void callbackWeight2(const std_msgs::Float64MultiArray::ConstPtr& msg);

  /**
   * @brief set pose measurement
   */
  void callbackPose1(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /**
   * @brief set pose measurement
   */
  void callbackPose2(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /**
   * @brief set poses measurement synchronized
   */
  void callbackPoses(const geometry_msgs::PoseStamped::ConstPtr& msg1,
                     const geometry_msgs::PoseStamped::ConstPtr& msg2);

  /**
   * @brief set twist measurement
   */
  void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg);

  /**
   * @brief set poseWithCovariance measurement
   */
  void callbackPose1WithCovariance(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  /**
   * @brief set poseWithCovariance measurement
   */
  void callbackPose2WithCovariance(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  /**
   * @brief set twistWithCovariance measurement
   */
  void callbackTwistWithCovariance(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);

  /**
   * @brief set initial_pose to current EKF pose
   */
  void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped& msg);

  /**
   * @brief initialization of EKF
   */
  void initEKF();

  /**
   * @brief compute EKF prediction
   */
  void predictKinematicsModel();

  /**
   * @brief compute EKF update with pose measurement
   * @param pose measurement value
   */
  void measurementUpdatePose(const geometry_msgs::PoseStamped& pose);

  /**
   * @brief compute EKF update with pose measurement
   * @param twist measurement value
   */
  void measurementUpdateTwist(const geometry_msgs::TwistStamped& twist);

  /**
   * @brief check whether a measurement value falls within the mahalanobis distance threshold
   * @param dist_max mahalanobis distance threshold
   * @param estimated current estimated state
   * @param measured measured state
   * @param estimated_cov current estimation covariance
   * @return whether it falls within the mahalanobis distance threshold
   */
  bool mahalanobisGate(const double& dist_max, const Eigen::MatrixXd& estimated, const Eigen::MatrixXd& measured,
                       const Eigen::MatrixXd& estimated_cov);

  /**
   * @brief get transform from frame_id
   */
  bool getTransformFromTF(std::string parent_frame, std::string child_frame,
                          geometry_msgs::TransformStamped& transform);

  /**
   * @brief normalize yaw angle
   * @param yaw yaw angle
   * @return normalized yaw
   */
  double normalizeYaw(const double& yaw);

  /**
   * @brief set current EKF estimation result to current_ekf_pose_ & current_ekf_twist_
   */
  void setCurrentResult();

  /**
   * @brief publish current EKF estimation result
   */
  void publishEstimateResult();

  /**
   * @brief for debug
   */
  void showCurrentX();

  /**
   * @brief for debug
   */
  void EKFStatus();

  /*
   * @brief for debug
   */
  std::tuple<Error, Error, Error> 
    calculateErrors(const geometry_msgs::PoseStamped& pose1, 
                    const geometry_msgs::PoseStamped& pose2);

  friend class EKFLocalizerTestSuite;  // for test code
};
