/*
 * Copyright 2023 ACCESS Lab (NC A&T SU). All rights reserved.
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
*/

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <novatel_oem7_msgs/BESTPOS.h>
#include <autoware_msgs/NDTStat.h>
#include <autoware_msgs/Lane.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

class SensorWeight
{
public:
  SensorWeight();
  ~SensorWeight();
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_weight1_;
  ros::Publisher pub_weight2_;
  ros::Subscriber sub_uncertainty1_;
  ros::Subscriber sub_uncertainty2_;
  
  /* parameters */
  double max_uncertainty1_;
  double max_uncertainty2_;

  void callbackUncertainty1(const autoware_msgs::NDTStat::ConstPtr& msg);
  void callbackUncertainty2(const novatel_oem7_msgs::BESTPOS::ConstPtr& msg);
  std_msgs::Float64MultiArray calculate_weights(double curr_value_, double max_value_);
};