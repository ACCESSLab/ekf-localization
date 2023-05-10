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

#include "ekf_localizer/ekf_localizer.h"

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) { if (show_debug_info_) { ROS_INFO(__VA_ARGS__); } }
#define DEBUG_PRINT_MAT(X) { if (show_debug_info_) { std::cout << #X << ": " << X << std::endl; } }

// clang-format on
EKFLocalizer::EKFLocalizer() : nh_(""), pnh_("~"), dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */)
{
  pnh_.param("show_debug_info", show_debug_info_, bool(false));
  pnh_.param("predict_frequency", ekf_rate_, double(50.0));
  ekf_dt_ = 1.0 / std::max(ekf_rate_, 0.1);
  pnh_.param("enable_yaw_bias_estimation", enable_yaw_bias_estimation_, bool(true));
  pnh_.param("extend_state_step", extend_state_step_, int(50));
  pnh_.param("pose_frame_id", pose_frame_id_, std::string("map"));
  pnh_.param("output_frame_id", output_frame_id_, std::string("base_link"));
  int method_type_tmp = 0;
  pnh_.getParam("method_type", method_type_tmp);
  method_type_ = static_cast<MethodType>(method_type_tmp);
  pnh_.param("use_synchronization", use_synchronization_, bool(false));
  
  /* pose measurement */
  pnh_.param("pose1_additional_delay", pose1_additional_delay_, double(0.0));
  pnh_.param("pose2_additional_delay", pose2_additional_delay_, double(0.0));
  pnh_.param("pose_measure_uncertainty_time", pose_measure_uncertainty_time_, double(0.01));
  
  pnh_.param("pose1_rate", pose1_rate_, double(10.0));               // used for covariance calculation
  pnh_.param("pose1_gate_dist", pose1_gate_dist_, double(10000.0));  // Mahalanobis limit
  pnh_.param("pose1_stddev_x", pose1_stddev_x_, double(0.05));
  pnh_.param("pose1_stddev_y", pose1_stddev_y_, double(0.05));
  pnh_.param("pose1_stddev_yaw", pose1_stddev_yaw_, double(0.035));
  
  pnh_.param("pose2_rate", pose2_rate_, double(10.0));               // used for covariance calculation
  pnh_.param("pose2_gate_dist", pose2_gate_dist_, double(10000.0));  // Mahalanobis limit
  pnh_.param("pose2_stddev_x", pose2_stddev_x_, double(0.05));
  pnh_.param("pose2_stddev_y", pose2_stddev_y_, double(0.05));
  pnh_.param("pose2_stddev_yaw", pose2_stddev_yaw_, double(0.035));
  
  pnh_.param("use_pose_with_covariance", use_pose_with_covariance_, bool(false));  
  
  /* twist measurement */
  pnh_.param("twist_additional_delay", twist_additional_delay_, double(0.0));
  pnh_.param("twist_rate", twist_rate_, double(10.0));               // used for covariance calculation
  pnh_.param("twist_gate_dist", twist_gate_dist_, double(10000.0));  // Mahalanobis limit
  pnh_.param("twist_stddev_vx", twist_stddev_vx_, double(0.2));
  pnh_.param("twist_stddev_wz", twist_stddev_wz_, double(0.03));
  pnh_.param("use_twist_with_covariance", use_twist_with_covariance_, bool(false));

  /* process noise */
  double proc_stddev_yaw_c, proc_stddev_yaw_bias_c, proc_stddev_vx_c, proc_stddev_wz_c;
  pnh_.param("proc_stddev_yaw_c", proc_stddev_yaw_c, double(0.005));
  pnh_.param("proc_stddev_yaw_bias_c", proc_stddev_yaw_bias_c, double(0.001));
  pnh_.param("proc_stddev_vx_c", proc_stddev_vx_c, double(2.0));
  pnh_.param("proc_stddev_wz_c", proc_stddev_wz_c, double(0.2));
  if (!enable_yaw_bias_estimation_)
  {
    proc_stddev_yaw_bias_c = 0.0;
  }

  /* convert to continuous to discrete */
  proc_cov_vx_d_ = std::pow(proc_stddev_vx_c, 2.0) * ekf_dt_;
  proc_cov_wz_d_ = std::pow(proc_stddev_wz_c, 2.0) * ekf_dt_;
  proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c, 2.0) * ekf_dt_;
  proc_cov_yaw_bias_d_ = std::pow(proc_stddev_yaw_bias_c, 2.0) * ekf_dt_;

  /* initialize ros system */

  timer_control_ = nh_.createTimer(ros::Duration(ekf_dt_), &EKFLocalizer::timerCallback, this);
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("ekf_pose", 1);
  pub_pose_cov_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose_with_covariance", 1);
  pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>("ekf_twist", 1);
  pub_twist_cov_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("ekf_twist_with_covariance", 1);
  pub_yaw_bias_ = pnh_.advertise<std_msgs::Float64>("estimated_yaw_bias", 1);
  sub_initialpose_ = nh_.subscribe("initialpose", 1, &EKFLocalizer::callbackInitialPose, this);
  sub_pose1_with_cov_ = nh_.subscribe("in_pose1_with_covariance", 1, &EKFLocalizer::callbackPose1WithCovariance, this);
  sub_pose2_with_cov_ = nh_.subscribe("in_pose2_with_covariance", 1, &EKFLocalizer::callbackPose2WithCovariance, this);
  sub_pose1_ = nh_.subscribe("in_pose1", 1, &EKFLocalizer::callbackPose1, this);
  sub_pose2_ = nh_.subscribe("in_pose2", 1, &EKFLocalizer::callbackPose2, this);
  sub_weight1_ = nh_.subscribe("in_weight1", 1, &EKFLocalizer::callbackWeight1, this);
  sub_weight2_ = nh_.subscribe("in_weight2", 1, &EKFLocalizer::callbackWeight2, this);
  sub_twist_with_cov_ = nh_.subscribe("in_twist_with_covariance", 1, &EKFLocalizer::callbackTwistWithCovariance, this);
  sub_twist_ = nh_.subscribe("in_twist", 1, &EKFLocalizer::callbackTwist, this);
  
  sub_pose1_sync_.subscribe(nh_, "in_pose1", 1);
  sub_pose2_sync_.subscribe(nh_, "in_pose2", 1);
  sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), sub_pose1_sync_, sub_pose2_sync_));
  sync_->registerCallback(boost::bind(&EKFLocalizer::callbackPoses, this, _1, _2));

  dim_x_ex_ = dim_x_ * extend_state_step_;

  initEKF();

  /* debug */
  pub_debug_ = pnh_.advertise<std_msgs::Float64MultiArray>("debug", 1);
  pub_measured_pose1_ = pnh_.advertise<geometry_msgs::PoseStamped>("debug/measured_pose1", 1);
  pub_measured_pose2_ = pnh_.advertise<geometry_msgs::PoseStamped>("debug/measured_pose2", 1);

  // ekf monitor
  pub_overlay_info_text_ = pnh_.advertise<jsk_rviz_plugins::OverlayText>("ekf_monitor/ekf_info_text", 1);
  pub_ekf_status_str_ = pnh_.advertise<std_msgs::String>("ekf_monitor/ekf_status_str", 1);
  pub_ekf_status_int_ = pnh_.advertise<std_msgs::Int8>("ekf_monitor/ekf_status_int", 1);

  ekf_lidar_gnss_text_.width  = 800;
  ekf_lidar_gnss_text_.height = 100;
  ekf_lidar_gnss_text_.left   = 10;
  ekf_lidar_gnss_text_.top    = 10;
  ekf_lidar_gnss_text_.text_size  = 16;
  ekf_lidar_gnss_text_.line_width = 2;

  ekf_lidar_gnss_text_.bg_color.r = 0.f;
  ekf_lidar_gnss_text_.bg_color.g = 0.f;
  ekf_lidar_gnss_text_.bg_color.b = 0.f;
  ekf_lidar_gnss_text_.bg_color.a = 0.2f;

  ekf_lidar_gnss_text_.text = "EKF MONITOR - LIDAR + GNSS OK";
  ekf_lidar_gnss_text_.fg_color.r = 0.0f;
  ekf_lidar_gnss_text_.fg_color.g = 1.f;
  ekf_lidar_gnss_text_.fg_color.b = 0.0f;
  ekf_lidar_gnss_text_.fg_color.a = 1.0f;

  ekf_lidar_text_ = ekf_lidar_gnss_text_;
  ekf_lidar_text_.text = "EKF MONITOR - ONLY LIDAR";
  ekf_lidar_text_.fg_color.r = 1.0f;
  ekf_lidar_text_.fg_color.g = 0.6f;

  ekf_gnss_text_ = ekf_lidar_gnss_text_;
  ekf_gnss_text_.text = "EKF MONITOR - ONLY GNSS";
  ekf_gnss_text_.fg_color.r = 1.0f;
  ekf_gnss_text_.fg_color.g = 0.6f;

  ekf_dead_reckoning_text_ = ekf_lidar_gnss_text_;
  ekf_dead_reckoning_text_.text = "EKF MONITOR - DEAD RECKONING";
  ekf_dead_reckoning_text_.fg_color.r = 1.0f;
  ekf_dead_reckoning_text_.fg_color.g = 0.0f;
};

EKFLocalizer::~EKFLocalizer(){};

/*
 * timerCallback
 */
void EKFLocalizer::timerCallback(const ros::TimerEvent& e)
{
  DEBUG_INFO("========================= timer called =========================");

  /* predict model in EKF */
  auto start = std::chrono::system_clock::now();
  DEBUG_INFO("------------------------- start prediction -------------------------");
  predictKinematicsModel();
  double elapsed =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count();
  DEBUG_INFO("[EKF] predictKinematicsModel calculation time = %f [ms]", elapsed * 1.0e-6);
  DEBUG_INFO("-------------------------- end prediction --------------------------\n");

  /* pose measurement update */
  if ((current_pose1_ptr_ != nullptr) && (method_type_ != MethodType::GNSS))
  {
    // update delay value for pose1
    pose_additional_delay_ = pose1_additional_delay_;
    pose_rate_ = pose1_rate_;
    pose_gate_dist_ = pose1_gate_dist_ * pose1_weight_;
    pose_stddev_x_ = pose1_stddev_x_;
    pose_stddev_y_ = pose1_stddev_y_;
    pose_stddev_yaw_ = pose1_stddev_yaw_;
    DEBUG_INFO("------------------------- start Pose1 -------------------------");
    start = std::chrono::system_clock::now();
    measurementUpdatePose(*current_pose1_ptr_);
    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count();
    DEBUG_INFO("[EKF] measurementUpdatePose calculation time = %f [ms]", elapsed * 1.0e-6);
    DEBUG_INFO("-------------------------- end Pose1 --------------------------\n");
    // if the measured data is ignored, activate flag for ekf monitor
    current_pose1_flag_ = true;
    if (!mahalanobis_gate_flag_)
    {
      current_pose1_flag_ = false;
      // accept measurement of next iteration after rejecting the current one
      mahalanobis_gate_flag_ = true;
    };
  }
  if ((current_pose2_ptr_ != nullptr) && (method_type_ != MethodType::LIDAR))
  {
    // update delay value for pose2
    pose_additional_delay_ = pose2_additional_delay_;
    pose_rate_ = pose2_rate_;
    pose_gate_dist_ = pose2_gate_dist_ * pose2_weight_;
    pose_stddev_x_ = pose2_stddev_x_;
    pose_stddev_y_ = pose2_stddev_y_;
    pose_stddev_yaw_ = pose2_stddev_yaw_;
    DEBUG_INFO("------------------------- start Pose2 -------------------------");
    start = std::chrono::system_clock::now();
    measurementUpdatePose(*current_pose2_ptr_);
    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count();
    DEBUG_INFO("[EKF] measurementUpdatePose calculation time = %f [ms]", elapsed * 1.0e-6);
    DEBUG_INFO("-------------------------- end Pose2 --------------------------\n");
    // if the measured data is ignored, activate flag for ekf monitor
    current_pose2_flag_ = true;
    if (!mahalanobis_gate_flag_)
    {
      current_pose2_flag_ = false;
      // accept measurement of next iteration after rejecting the current one
      mahalanobis_gate_flag_ = true;
    };
  }

  /* twist measurement update */
  if (current_twist_ptr_ != nullptr)
  {
    DEBUG_INFO("------------------------- start twist -------------------------");
    start = std::chrono::system_clock::now();
    measurementUpdateTwist(*current_twist_ptr_);
    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count();
    DEBUG_INFO("[EKF] measurementUpdateTwist calculation time = %f [ms]", elapsed * 1.0e-6);
    DEBUG_INFO("-------------------------- end twist --------------------------\n");
  }

  /* set current pose, twist */
  setCurrentResult();

  /* publish ekf result */
  publishEstimateResult();
}

void EKFLocalizer::showCurrentX()
{
  if (show_debug_info_)
  {
    Eigen::MatrixXd X(dim_x_, 1);
    ekf_.getLatestX(X);
    DEBUG_PRINT_MAT(X.transpose());
  }
}

/*
 * setCurrentResult
 */
void EKFLocalizer::setCurrentResult()
{
  current_ekf_pose_.header.frame_id = pose_frame_id_;
  current_ekf_pose_.header.stamp = ros::Time::now();
  current_ekf_pose_.pose.position.x = ekf_.getXelement(IDX::X);
  current_ekf_pose_.pose.position.y = ekf_.getXelement(IDX::Y);

  tf2::Quaternion q_tf;
  double roll, pitch, yaw;
  if (current_pose1_ptr_ != nullptr)
  {
    current_ekf_pose_.pose.position.z = current_pose1_ptr_->pose.position.z;
    tf2::fromMsg(current_pose1_ptr_->pose.orientation, q_tf); /* use Pose pitch and roll */
    tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
  }
  else
  {
    current_ekf_pose_.pose.position.z = 0.0;
    roll = 0;
    pitch = 0;
  }
  yaw = ekf_.getXelement(IDX::YAW) + ekf_.getXelement(IDX::YAWB);
  q_tf.setRPY(roll, pitch, yaw);
  tf2::convert(q_tf, current_ekf_pose_.pose.orientation);

  current_ekf_twist_.header.frame_id = "base_link";
  current_ekf_twist_.header.stamp = ros::Time::now();
  current_ekf_twist_.twist.linear.x = ekf_.getXelement(IDX::VX);
  current_ekf_twist_.twist.angular.z = ekf_.getXelement(IDX::WZ);
}

/*
 * broadcastTF
 */
void EKFLocalizer::broadcastTF()
{
  if (current_ekf_pose_.header.frame_id == "")
  {
    return;
  }

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header = current_ekf_pose_.header;
  transformStamped.child_frame_id = output_frame_id_;
  transformStamped.transform.translation.x = current_ekf_pose_.pose.position.x;
  transformStamped.transform.translation.y = current_ekf_pose_.pose.position.y;
  transformStamped.transform.translation.z = current_ekf_pose_.pose.position.z;

  transformStamped.transform.rotation.x = current_ekf_pose_.pose.orientation.x;
  transformStamped.transform.rotation.y = current_ekf_pose_.pose.orientation.y;
  transformStamped.transform.rotation.z = current_ekf_pose_.pose.orientation.z;
  transformStamped.transform.rotation.w = current_ekf_pose_.pose.orientation.w;

  tf_br_.sendTransform(transformStamped);
}

/*
 * getTransformFromTF
 */
bool EKFLocalizer::getTransformFromTF(std::string parent_frame, std::string child_frame,
                                      geometry_msgs::TransformStamped& transform)
{
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  ros::Duration(0.1).sleep();
  if (parent_frame.front() == '/')
    parent_frame.erase(0, 1);
  if (child_frame.front() == '/')
    child_frame.erase(0, 1);

  for (int i = 0; i < 50; ++i)
  {
    try
    {
      transform = tf_buffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
      return true;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(0.1).sleep();
    }
  }
  return false;
}

/*
 * callbackInitialPose
 */
void EKFLocalizer::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped& initialpose)
{
  geometry_msgs::TransformStamped transform;
  if (!getTransformFromTF(pose_frame_id_, initialpose.header.frame_id, transform))
  {
    ROS_ERROR("[EKF] TF transform failed. parent = %s, child = %s", pose_frame_id_.c_str(),
              initialpose.header.frame_id.c_str());
  };

  Eigen::MatrixXd X(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  X(IDX::X) = initialpose.pose.pose.position.x + transform.transform.translation.x;
  X(IDX::Y) = initialpose.pose.pose.position.y + transform.transform.translation.y;
  X(IDX::YAW) = tf2::getYaw(initialpose.pose.pose.orientation) + tf2::getYaw(transform.transform.rotation);
  X(IDX::YAWB) = 0.0;
  X(IDX::VX) = 0.0;
  X(IDX::WZ) = 0.0;

  P(IDX::X, IDX::X) = initialpose.pose.covariance[0];
  P(IDX::Y, IDX::Y) = initialpose.pose.covariance[6 + 1];
  P(IDX::YAW, IDX::YAW) = initialpose.pose.covariance[6 * 5 + 5];
  P(IDX::YAWB, IDX::YAWB) = 0.0001;
  P(IDX::VX, IDX::VX) = 0.01;
  P(IDX::WZ, IDX::WZ) = 0.01;

  ekf_.init(X, P, extend_state_step_);
};

/*
 * callbackWeight1
 */
void EKFLocalizer::callbackWeight1(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  pose1_weight_ = 1.0; // keep open the gate when using only one source of localization
  if (method_type_ == MethodType::LIDAR_GNSS)
  {
    pose1_weight_ = msg->data[0];
  }
};

/*
 * callbackWeight2
 */
void EKFLocalizer::callbackWeight2(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  pose2_weight_ = 1.0; // keep open the gate when using only one source of localization
  if (method_type_ == MethodType::LIDAR_GNSS)
  {
    pose2_weight_ = msg->data[0];
  }
};

/*
 * callbackPose1
 */
void EKFLocalizer::callbackPose1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if ((!use_pose_with_covariance_) && (!use_synchronization_)) 
  {
    current_pose1_.header = msg->header;
    current_pose1_.pose = msg->pose;
    if (current_pose1_.pose.position.x < 1e5)
      current_pose1_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(*msg);
  }
};

/*
 * callbackPose2
 */
void EKFLocalizer::callbackPose2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if ((!use_pose_with_covariance_) && (!use_synchronization_))  
  {
    current_pose2_.header = msg->header;
    current_pose2_.pose = msg->pose;
    if (current_pose2_.pose.position.x < 1e5)
      current_pose2_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(*msg);
  }
};

/*
 * callbackPoses
 */
void EKFLocalizer::callbackPoses(const geometry_msgs::PoseStamped::ConstPtr& msg1,
                                 const geometry_msgs::PoseStamped::ConstPtr& msg2)
{
  if ((!use_pose_with_covariance_) && (use_synchronization_))  
  {
    // callback data from pose1
    current_pose1_.header = msg1->header;
    current_pose1_.pose   = msg1->pose;
    if (current_pose1_.pose.position.x < 1e5)
      current_pose1_ptr_  = std::make_shared<geometry_msgs::PoseStamped>(*msg1);
    
    // callback data from pose2
    current_pose2_.header = msg2->header;
    current_pose2_.pose   = msg2->pose;
    if (current_pose2_.pose.position.x < 1e5)
      current_pose2_ptr_  = std::make_shared<geometry_msgs::PoseStamped>(*msg2);
    
    DEBUG_INFO("-------------------- start synchronization --------------------");
    DEBUG_INFO("Received synchronized messages:");
    DEBUG_INFO("Pose1: x=%f, y=%f, z=%f", msg1->pose.position.x, msg1->pose.position.y, msg1->pose.position.z);
    DEBUG_INFO("Pose2: x=%f, y=%f, z=%f", msg2->pose.position.x, msg2->pose.position.y, msg2->pose.position.z);
        
    // compare the synchronized messages with the original messages
    DEBUG_INFO("Original message from topic in_pose1: x=%f, y=%f, z=%f", current_pose1_.pose.position.x, current_pose1_.pose.position.y, current_pose1_.pose.position.z);
    DEBUG_INFO("Original message from topic in_pose2: x=%f, y=%f, z=%f", current_pose2_.pose.position.x, current_pose2_.pose.position.y, current_pose2_.pose.position.z);
      
    // calculate the time difference between the two messages
    double time_diff = (msg1->header.stamp - msg2->header.stamp).toSec();
    DEBUG_INFO("Time difference between messages: %f seconds", time_diff);
    DEBUG_INFO("--------------------- end synchronization ---------------------");
  }
};

/*
 * callbackPose1WithCovariance
 */
void EKFLocalizer::callbackPose1WithCovariance(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  if (use_pose_with_covariance_)
  {
    current_pose1_.header = msg->header;
    current_pose1_.pose = msg->pose.pose;
    if (current_pose1_.pose.position.x < 1e5)
    {
      current_pose1_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(current_pose1_);
      current_pose_covariance_ = msg->pose.covariance;
    }
  }
};

/*
 * callbackPose2WithCovariance
 */
void EKFLocalizer::callbackPose2WithCovariance(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  if (use_pose_with_covariance_)
  {
    current_pose2_.header = msg->header;
    current_pose2_.pose = msg->pose.pose;
    if (current_pose2_.pose.position.x < 1e5)
    {
      current_pose2_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(current_pose2_);
      current_pose_covariance_ = msg->pose.covariance;
    }
  }
};

/*
 * callbackTwist
 */
void EKFLocalizer::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  if (!use_twist_with_covariance_)
  {
    current_twist_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(*msg);
  }
};

/*
 * callbackTwistWithCovariance
 */
void EKFLocalizer::callbackTwistWithCovariance(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg)
{
  if (use_twist_with_covariance_)
  {
    geometry_msgs::TwistStamped twist;
    twist.header = msg->header;
    twist.twist = msg->twist.twist;
    current_twist_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(twist);
    current_twist_covariance_ = msg->twist.covariance;
  }
};

/*
 * initEKF
 */
void EKFLocalizer::initEKF()
{
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E15;  // for x & y
  P(IDX::YAW, IDX::YAW) = 50.0;                                            // for yaw
  P(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_;                          // for yaw bias
  P(IDX::VX, IDX::VX) = 1000.0;                                            // for vx
  P(IDX::WZ, IDX::WZ) = 50.0;                                              // for wz

  ekf_.init(X, P, extend_state_step_);
}

/*
 * predictKinematicsModel
 */
void EKFLocalizer::predictKinematicsModel()
{
  /*  == Nonlinear model ==
   *
   * x_{k+1}   = x_k + vx_k * cos(yaw_k + b_k) * dt
   * y_{k+1}   = y_k + vx_k * sin(yaw_k + b_k) * dt
   * yaw_{k+1} = yaw_k + (wz_k) * dt
   * b_{k+1}   = b_k
   * vx_{k+1}  = vz_k
   * wz_{k+1}  = wz_k
   *
   * (b_k : yaw_bias_k)
   */

  /*  == Linearized model ==
   *
   * A = [ 1, 0, -vx*sin(yaw+b)*dt, -vx*sin(yaw+b)*dt, cos(yaw+b)*dt,  0]
   *     [ 0, 1,  vx*cos(yaw+b)*dt,  vx*cos(yaw+b)*dt, sin(yaw+b)*dt,  0]
   *     [ 0, 0,                 1,                 0,             0, dt]
   *     [ 0, 0,                 0,                 1,             0,  0]
   *     [ 0, 0,                 0,                 0,             1,  0]
   *     [ 0, 0,                 0,                 0,             0,  1]
   */

  Eigen::MatrixXd X_curr(dim_x_, 1);  // curent state
  Eigen::MatrixXd X_next(dim_x_, 1);  // predicted state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  Eigen::MatrixXd P_curr;
  ekf_.getLatestP(P_curr);

  const int d_dim_x = dim_x_ex_ - dim_x_;
  const double yaw = X_curr(IDX::YAW);
  const double yaw_bias = X_curr(IDX::YAWB);
  const double vx = X_curr(IDX::VX);
  const double wz = X_curr(IDX::WZ);
  const double dt = ekf_dt_;

  /* Update for latest state */
  X_next(IDX::X) = X_curr(IDX::X) + vx * cos(yaw + yaw_bias) * dt;  // dx = v * cos(yaw)
  X_next(IDX::Y) = X_curr(IDX::Y) + vx * sin(yaw + yaw_bias) * dt;  // dy = v * sin(yaw)
  X_next(IDX::YAW) = X_curr(IDX::YAW) + (wz)*dt;                    // dyaw = omega + omega_bias
  X_next(IDX::YAWB) = yaw_bias;
  X_next(IDX::VX) = vx;
  X_next(IDX::WZ) = wz;

  X_next(IDX::YAW) = std::atan2(std::sin(X_next(IDX::YAW)), std::cos(X_next(IDX::YAW)));

  /* Set A matrix for latest state */
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  A(IDX::X, IDX::YAW) = -vx * sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::YAWB) = -vx * sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::VX) = cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAW) = vx * cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAWB) = vx * cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::VX) = sin(yaw + yaw_bias) * dt;
  A(IDX::YAW, IDX::WZ) = dt;

  const double dvx = std::sqrt(P_curr(IDX::VX, IDX::VX));
  const double dyaw = std::sqrt(P_curr(IDX::YAW, IDX::YAW));

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  if (dvx < 10.0 && dyaw < 1.0)
  {
    // auto covariance calculate for x, y assuming vx & yaw estimation covariance is small

    /* Set covariance matrix Q for process noise. Calc Q by velocity and yaw angle covariance :
     dx = Ax + Jp*w -> Q = Jp*w_cov*Jp'          */
    Eigen::MatrixXd Jp = Eigen::MatrixXd::Zero(2, 2);  // coeff of deviation of vx & yaw
    Jp << cos(yaw), -vx * sin(yaw), sin(yaw), vx * cos(yaw);
    Eigen::MatrixXd Q_vx_yaw = Eigen::MatrixXd::Zero(2, 2);  // cov of vx and yaw

    Q_vx_yaw(0, 0) = P_curr(IDX::VX, IDX::VX) * dt;        // covariance of vx - vx
    Q_vx_yaw(1, 1) = P_curr(IDX::YAW, IDX::YAW) * dt;      // covariance of yaw - yaw
    Q_vx_yaw(0, 1) = P_curr(IDX::VX, IDX::YAW) * dt;       // covariance of vx - yaw
    Q_vx_yaw(1, 0) = P_curr(IDX::YAW, IDX::VX) * dt;       // covariance of yaw - vx
    Q.block(0, 0, 2, 2) = Jp * Q_vx_yaw * Jp.transpose();  // for pos_x & pos_y
  }
  else
  {
    // vx & vy is not converged yet, set constant value.
    Q(IDX::X, IDX::X) = 0.05;
    Q(IDX::Y, IDX::Y) = 0.05;
  }

  Q(IDX::YAW, IDX::YAW) = proc_cov_yaw_d_;         // for yaw
  Q(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_;  // for yaw bias
  Q(IDX::VX, IDX::VX) = proc_cov_vx_d_;            // for vx
  Q(IDX::WZ, IDX::WZ) = proc_cov_wz_d_;            // for wz

  ekf_.predictWithDelay(X_next, A, Q);

  // debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdatePose
 */
void EKFLocalizer::measurementUpdatePose(const geometry_msgs::PoseStamped& pose)
{
  if (pose.header.frame_id != pose_frame_id_)
  {
    ROS_WARN_DELAYED_THROTTLE(2, "pose frame_id is %s, but pose_frame is set as %s. They must be same.",
                              pose.header.frame_id.c_str(), pose_frame_id_.c_str());
  }
  Eigen::MatrixXd X_curr(dim_x_, 1);  // curent state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 3;  // pos_x, pos_y, yaw, depending on Pose output
  const ros::Time t_curr = ros::Time::now();

  /* Calculate delay step */
  double delay_time = (t_curr - pose.header.stamp).toSec() + pose_additional_delay_;
  if (delay_time < 0.0)
  {
    delay_time = 0.0;
    ROS_WARN_DELAYED_THROTTLE(1.0, "Pose time stamp is inappropriate, set delay to 0[s]. delay = %f", delay_time);
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1)
  {
    ROS_WARN_DELAYED_THROTTLE(1.0,
                              "Pose delay exceeds the compensation limit, ignored. delay: %f[s], limit = "
                              "extend_state_step * ekf_dt : %f [s]",
                              delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  DEBUG_INFO("delay_time: %f [s]", delay_time);

  /* Set yaw */
  const double yaw_curr = ekf_.getXelement((unsigned int)(delay_step * dim_x_ + IDX::YAW));
  double yaw = tf2::getYaw(pose.pose.orientation);
  const double ekf_yaw = ekf_.getXelement(delay_step * dim_x_ + IDX::YAW);
  const double yaw_error = normalizeYaw(yaw - ekf_yaw);  // normalize the error not to exceed 2 pi
  yaw = yaw_error + ekf_yaw;

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << pose.pose.position.x, pose.pose.position.y, yaw;

  if (isnan(y.array()).any() || isinf(y.array()).any())
  {
    ROS_WARN("[EKF] pose measurement matrix includes NaN of Inf. ignore update. check pose message.");
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::X), ekf_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw;
  Eigen::MatrixXd P_curr, P_y;
  ekf_.getLatestP(P_curr);
  P_y = P_curr.block(0, 0, dim_y, dim_y);
  DEBUG_INFO("-------------------- start mahalanobis pose --------------------");
  if (!mahalanobisGate(pose_gate_dist_, y_ekf, y, P_y))
  {
    ROS_WARN_DELAYED_THROTTLE(2.0, "[EKF] Pose measurement update, mahalanobis distance is over limit. ignore "
                                   "measurement data.");
    mahalanobis_gate_flag_ = false;
    return;
  }
  DEBUG_INFO("--------------------- end mahalanobis pose ---------------------");
  
  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::X) = 1.0;    // for pos x
  C(1, IDX::Y) = 1.0;    // for pos y
  C(2, IDX::YAW) = 1.0;  // for yaw

  /* Set measurement noise covariancs */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (use_pose_with_covariance_)
  {
    R(0, 0) = current_pose_covariance_.at(0);   // x - x
    R(0, 1) = current_pose_covariance_.at(1);   // x - y
    R(0, 2) = current_pose_covariance_.at(5);   // x - yaw
    R(1, 0) = current_pose_covariance_.at(6);   // y - x
    R(1, 1) = current_pose_covariance_.at(7);   // y - y
    R(1, 2) = current_pose_covariance_.at(11);  // y - yaw
    R(2, 0) = current_pose_covariance_.at(30);  // yaw - x
    R(2, 1) = current_pose_covariance_.at(31);  // yaw - y
    R(2, 2) = current_pose_covariance_.at(35);  // yaw - yaw
  }
  else
  {
    const double ekf_yaw = ekf_.getXelement(IDX::YAW);
    const double vx = ekf_.getXelement(IDX::VX);
    const double wz = ekf_.getXelement(IDX::WZ);
    const double cov_pos_x = std::pow(pose_measure_uncertainty_time_ * vx * cos(ekf_yaw), 2.0);
    const double cov_pos_y = std::pow(pose_measure_uncertainty_time_ * vx * sin(ekf_yaw), 2.0);
    const double cov_yaw = std::pow(pose_measure_uncertainty_time_ * wz, 2.0);
    R(0, 0) = std::pow(pose_stddev_x_, 2) + cov_pos_x;  // pos_x
    R(1, 1) = std::pow(pose_stddev_y_, 2) + cov_pos_y;  // pos_y
    R(2, 2) = std::pow(pose_stddev_yaw_, 2) + cov_yaw;  // yaw
  }

  /* In order to avoid a large change at the time of updating, measuremeent update is performed by dividing at every
   * step. */
  R *= (ekf_rate_ / pose_rate_);
  
  ekf_.updateWithDelay(y, C, R, delay_step);

  // debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdateTwist
 */
void EKFLocalizer::measurementUpdateTwist(const geometry_msgs::TwistStamped& twist)
{
  if (twist.header.frame_id != "base_link")
  {
    ROS_WARN_DELAYED_THROTTLE(2.0, "twist frame_id must be base_link");
  }

  Eigen::MatrixXd X_curr(dim_x_, 1);  // curent state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 2;  // vx, wz
  const ros::Time t_curr = ros::Time::now();

  /* Calculate delay step */
  double delay_time = (t_curr - twist.header.stamp).toSec() + twist_additional_delay_;
  if (delay_time < 0.0)
  {
    ROS_WARN_DELAYED_THROTTLE(1.0, "Twist time stamp is inappropriate (delay = %f [s]), set delay to 0[s].",
                              delay_time);
    delay_time = 0.0;
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1)
  {
    ROS_WARN_DELAYED_THROTTLE(1.0,
                              "Twist delay exceeds the compensation limit, ignored. delay: %f[s], limit = "
                              "extend_state_step * ekf_dt : %f [s]",
                              delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  DEBUG_INFO("delay_time: %f [s]", delay_time);

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << twist.twist.linear.x, twist.twist.angular.z;

  if (isnan(y.array()).any() || isinf(y.array()).any())
  {
    ROS_WARN("[EKF] twist measurement matrix includes NaN of Inf. ignore update. check twist message.");
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::VX), ekf_.getXelement(delay_step * dim_x_ + IDX::WZ);
  Eigen::MatrixXd P_curr, P_y;
  ekf_.getLatestP(P_curr);
  P_y = P_curr.block(4, 4, dim_y, dim_y);
  DEBUG_INFO("-------------------- start mahalanobis twist --------------------");
  if (!mahalanobisGate(twist_gate_dist_, y_ekf, y, P_y))
  {
    ROS_WARN_DELAYED_THROTTLE(2.0, "[EKF] Twist measurement update, mahalanobis distance is over limit. ignore "
                                   "measurement data.");
    return;
  }
  DEBUG_INFO("--------------------- end mahalanobis twist ---------------------");

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::VX) = 1.0;  // for vx
  C(1, IDX::WZ) = 1.0;  // for wz

  /* Set measurement noise covariancs */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (use_twist_with_covariance_)
  {
    R(0, 0) = current_twist_covariance_.at(0);   // vx - vx
    R(0, 1) = current_twist_covariance_.at(5);   // vx - wz
    R(1, 0) = current_twist_covariance_.at(30);  // wz - vx
    R(1, 1) = current_twist_covariance_.at(35);  // wz - wz
  }
  else
  {
    R(0, 0) = twist_stddev_vx_ * twist_stddev_vx_ * ekf_dt_;  // for vx
    R(1, 1) = twist_stddev_wz_ * twist_stddev_wz_ * ekf_dt_;  // for wz
  }

  /* In order to avoid a large change by update, measurement update is performed by dividing at every step. */
  R *= (ekf_rate_ / twist_rate_);

  ekf_.updateWithDelay(y, C, R, delay_step);

  // debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
};

/*
 * mahalanobisGate
 */
bool EKFLocalizer::mahalanobisGate(const double& dist_max, const Eigen::MatrixXd& x, const Eigen::MatrixXd& obj_x,
                                   const Eigen::MatrixXd& cov)
{
  Eigen::MatrixXd mahalanobis_squared = (x - obj_x).transpose() * cov.inverse() * (x - obj_x);
  DEBUG_INFO("measurement update: mahalanobis = %f, gate limit = %f", std::sqrt(mahalanobis_squared(0)), dist_max);
  if (mahalanobis_squared(0) > dist_max * dist_max)
  {
    return false;
  }

  return true;
}

/*
 * publishEstimateResult
 */
void EKFLocalizer::publishEstimateResult()
{
  ros::Time current_time = ros::Time::now();
  Eigen::MatrixXd X(dim_x_, 1);
  Eigen::MatrixXd P(dim_x_, dim_x_);
  ekf_.getLatestX(X);
  ekf_.getLatestP(P);

  /* publish latest pose */
  pub_pose_.publish(current_ekf_pose_);

  /* publish latest pose with covariance */
  geometry_msgs::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = current_time;
  pose_cov.header.frame_id = current_ekf_pose_.header.frame_id;
  pose_cov.pose.pose = current_ekf_pose_.pose;
  pose_cov.pose.covariance[0] = P(IDX::X, IDX::X);
  pose_cov.pose.covariance[1] = P(IDX::X, IDX::Y);
  pose_cov.pose.covariance[5] = P(IDX::X, IDX::YAW);
  pose_cov.pose.covariance[6] = P(IDX::Y, IDX::X);
  pose_cov.pose.covariance[7] = P(IDX::Y, IDX::Y);
  pose_cov.pose.covariance[11] = P(IDX::Y, IDX::YAW);
  pose_cov.pose.covariance[30] = P(IDX::YAW, IDX::X);
  pose_cov.pose.covariance[31] = P(IDX::YAW, IDX::Y);
  pose_cov.pose.covariance[35] = P(IDX::YAW, IDX::YAW);
  pub_pose_cov_.publish(pose_cov);

  /* publish latest twist */
  pub_twist_.publish(current_ekf_twist_);

  /* publish latest twist with covariance */
  geometry_msgs::TwistWithCovarianceStamped twist_cov;
  twist_cov.header.stamp = current_time;
  twist_cov.header.frame_id = current_ekf_twist_.header.frame_id;
  twist_cov.twist.twist = current_ekf_twist_.twist;
  twist_cov.twist.covariance[0] = P(IDX::VX, IDX::VX);
  twist_cov.twist.covariance[5] = P(IDX::VX, IDX::WZ);
  twist_cov.twist.covariance[30] = P(IDX::WZ, IDX::VX);
  twist_cov.twist.covariance[35] = P(IDX::WZ, IDX::WZ);
  pub_twist_cov_.publish(twist_cov);

  /* Send transform of pose */
  broadcastTF();

  /* publish yaw bias */
  std_msgs::Float64 yawb;
  yawb.data = X(IDX::YAWB);
  pub_yaw_bias_.publish(yawb);

  /* ekf status publisher */
  EKFStatus();
  
  /* debug measured pose and publish */
  double RAD2DEG = 180.0 / M_PI;
  double pose1_yaw, pose2_yaw;
  geometry_msgs::PoseStamped p1, p2;
  if (current_pose1_ptr_ != nullptr)
  {
    p1 = *current_pose1_ptr_;
    p1.header.stamp = current_time;
    pub_measured_pose1_.publish(p1);
    pose1_yaw = tf2::getYaw(current_pose1_ptr_->pose.orientation) * RAD2DEG;
  }
  if (current_pose2_ptr_ != nullptr)
  {
    p2 = *current_pose2_ptr_;
    p2.header.stamp = current_time;
    p2.pose.position.z = current_ekf_pose_.pose.position.z; // make pose2 height the same as in pose1
    pub_measured_pose2_.publish(p2);
    pose2_yaw = tf2::getYaw(current_pose2_ptr_->pose.orientation) * RAD2DEG;
  }
  if (ekf_status_ == MethodType::LIDAR_GNSS)
  {
    // check the root mean square error between the positions and yaw angles 
    std::tuple<Error, Error, Error> errors = calculateErrors(p1, p2);
    DEBUG_INFO("ABSE Pos: %f m, Yaw: %f deg", abse.pos, abse.yaw);
    DEBUG_INFO("RMSE Pos: %f m, Yaw: %f deg", rmse.pos, rmse.yaw);
    DEBUG_INFO("MAE Pos: %f m, Yaw: %f deg", mae.pos, mae.yaw);
  } 

  std_msgs::Float64MultiArray msg;
  msg.data.push_back(X(IDX::YAW) * RAD2DEG);  // [0] ekf yaw angle
  msg.data.push_back(pose1_yaw);              // [1] measurement1 yaw angle
  msg.data.push_back(pose2_yaw);              // [2] measurement2 yaw angle
  msg.data.push_back(X(IDX::YAWB) * RAD2DEG); // [3] yaw bias
  msg.data.push_back(abse.pos);               // [4] abse position (when msf active)
  msg.data.push_back(abse.yaw);               // [5] abse yaw (when msf active)
  msg.data.push_back(rmse.pos);               // [6] rmse position (when msf active)
  msg.data.push_back(rmse.yaw);               // [7] rmse yaw (when msf active)
  msg.data.push_back(mae.pos);                // [8] mae position (when msf active)
  msg.data.push_back(mae.yaw);                // [9] mae yaw (when msf active)
  pub_debug_.publish(msg);

  // change pose flags to false, they will be true again when new data received
  current_pose1_flag_ = false;
  current_pose2_flag_ = false;
}

double EKFLocalizer::normalizeYaw(const double& yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

void EKFLocalizer::EKFStatus()
{
  // publish ekf status
  jsk_rviz_plugins::OverlayText rviz_info_text;
  std_msgs::String ekf_status_msg;
  std_msgs::Int8 ekf_status_num;
  // here the counter_max_ is defined at 10 (0.2 sec if pred_frequency = 50 hz)
  int counter_max_ = 10;
  
  if ((current_pose1_flag_) && (current_pose2_flag_))
  {
    ekf_status_ = MethodType::LIDAR_GNSS;
    ekf_status_num.data = 3;
    counter_pose1_status_ = 0;
    counter_pose2_status_ = 0;
  }
  else if ((current_pose1_flag_) && (!current_pose2_flag_))
  {
    counter_pose1_status_ = 0;
    counter_pose2_status_ += 1;
    if (counter_pose2_status_ > counter_max_)
    {
      ekf_status_ = MethodType::LIDAR;
      ekf_status_num.data = 1;
      counter_pose2_status_ = counter_max_; // keep this counter at low value
    }
  }
  else if ((!current_pose1_flag_) && (current_pose2_flag_))
  {
    counter_pose1_status_ += 1;
    counter_pose2_status_ = 0;
    if (counter_pose1_status_ > counter_max_)
    {
      ekf_status_ = MethodType::GNSS;
      ekf_status_num.data = 2;
      counter_pose1_status_ = counter_max_; // keep this counter at low value
    }
  }
  else
  {
    counter_pose1_status_ += 1;
    counter_pose2_status_ += 1;
    if ((counter_pose1_status_ > counter_max_) && (counter_pose2_status_ > counter_max_))
    {
      ekf_status_ = MethodType::DEAD_RECKONING;
      ekf_status_num.data = 0;
      counter_pose1_status_ = counter_max_; // keep this counter at low value
      counter_pose2_status_ = counter_max_; // keep this counter at low value
    }
  }

  ekf_status_msg.data = ekf_status_names[ekf_status_];
  
  switch (ekf_status_)
  {
    case MethodType::LIDAR:
      rviz_info_text = ekf_lidar_text_;
      break;
    case MethodType::GNSS:
      rviz_info_text = ekf_gnss_text_;
      break;
    case MethodType::LIDAR_GNSS:
      rviz_info_text = ekf_lidar_gnss_text_;
      break;
    default:
      rviz_info_text = ekf_dead_reckoning_text_;
      break;
  }
  pub_overlay_info_text_.publish(rviz_info_text);
  pub_ekf_status_str_.publish(ekf_status_msg);
  pub_ekf_status_int_.publish(ekf_status_num);
}

std::tuple<EKFLocalizer::Error, EKFLocalizer::Error, EKFLocalizer::Error> 
  EKFLocalizer::calculateErrors(const geometry_msgs::PoseStamped& pose1, 
                                const geometry_msgs::PoseStamped& pose2)
{
  current_pose1_vec_.push_back(pose1);
  current_pose2_vec_.push_back(pose2);
    
  int n = current_pose1_vec_.size();
  
  for(int i=0; i<n; ++i)
  {
    double error_x = current_pose1_vec_[i].pose.position.x - current_pose2_vec_[i].pose.position.x;
    double error_y = current_pose1_vec_[i].pose.position.y - current_pose2_vec_[i].pose.position.y;
    abse.pos = sqrt(error_x * error_x + error_y * error_y);

    double pose1_yaw = tf2::getYaw(current_pose1_vec_[i].pose.orientation);
    double pose2_yaw = tf2::getYaw(current_pose2_vec_[i].pose.orientation);
    abse.yaw = fabs(pose1_yaw - pose2_yaw);
    abse.yaw = fmod(abse.yaw + M_PI, 2.0 * M_PI) - M_PI; // ensure that the error is within [-pi, pi]

    rmse.pos += (abse.pos * abse.pos);
    rmse.yaw += (abse.yaw * abse.yaw);
    mae.pos += fabs(abse.pos);
    mae.yaw += fabs(abse.yaw);
  }

  rmse.pos = sqrt(rmse.pos / n);
  rmse.yaw = sqrt(rmse.yaw / n);
  mae.pos = mae.pos / n;
  mae.yaw = mae.yaw / n;

  return std::make_tuple(abse, rmse, mae);
}