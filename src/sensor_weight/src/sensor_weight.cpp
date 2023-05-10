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

#include "sensor_weight/sensor_weight.h"

SensorWeight::SensorWeight(): nh_(""), pnh_("~")
{
  pnh_.param<double>("max_uncertainty1", max_uncertainty1_, double(500.0));
  pnh_.param<double>("max_uncertainty2", max_uncertainty2_, double(2.0));
  
  pub_weight1_ = nh_.advertise<std_msgs::Float64MultiArray>("weight/ndt", 10);
  pub_weight2_ = nh_.advertise<std_msgs::Float64MultiArray>("weight/gnss", 10);
  sub_uncertainty1_ = nh_.subscribe("ndt_stat", 10, &SensorWeight::callbackUncertainty1, this);
  sub_uncertainty2_ = nh_.subscribe("novatel/oem7/bestpos", 10, &SensorWeight::callbackUncertainty2, this);
};

SensorWeight::~SensorWeight(){};

void SensorWeight::callbackUncertainty1(const autoware_msgs::NDTStat::ConstPtr& msg)
{
  double max_value_ = max_uncertainty1_;
  double curr_value_ = msg->score;

  std_msgs::Float64MultiArray weight;
  weight = calculate_weights(curr_value_, max_value_);
  
  pub_weight1_.publish(weight);
}

void SensorWeight::callbackUncertainty2(const novatel_oem7_msgs::BESTPOS::ConstPtr& msg)
{
  double max_value_ = max_uncertainty2_;
  double curr_value_ = sqrt(msg->lat_stdev * msg->lat_stdev + msg->lon_stdev * msg->lon_stdev);

  std_msgs::Float64MultiArray weight;
  weight = calculate_weights(curr_value_, max_value_);
  
  pub_weight2_.publish(weight);
}

std_msgs::Float64MultiArray SensorWeight::calculate_weights(double curr_value_, double max_value_) {
  std_msgs::Float64MultiArray weight;
  weight.data.resize(2);
  weight.data[0] = std::min(1.0, std::max(0.0, 1.0 - (curr_value_ / max_value_)));
  weight.data[1] = curr_value_;
  return weight;
}