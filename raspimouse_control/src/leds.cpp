// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright 2021 RT Corporation
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
 * This program is based on
 * https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/src/leds.cpp
 * which is BSD 3-Clause "New" or "Revised" License.
 *
 * Copyright (c) 2017, Ryuichi Ueda
 * https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/LICENSE
 */

#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include "raspimouse_msgs/LedValues.h"

std::string led0_device_file = "/dev/rtled0";
std::string led1_device_file = "/dev/rtled1";
std::string led2_device_file = "/dev/rtled2";
std::string led3_device_file = "/dev/rtled3";

void output(std::ofstream* ofs, bool input)
{
  *ofs << (input ? '1' : '0') << std::endl;
}

void cb(const raspimouse_msgs::LedValues::ConstPtr& msg)
{
  std::ofstream ofs0(led0_device_file);
  output(&ofs0, msg->right_side);
  std::ofstream ofs1(led1_device_file);
  output(&ofs1, msg->right_forward);
  std::ofstream ofs2(led2_device_file);
  output(&ofs2, msg->left_forward);
  std::ofstream ofs3(led3_device_file);
  output(&ofs3, msg->left_side);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "raspimouse_leds_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("device_file_0", led0_device_file);
  pnh.getParam("device_file_1", led1_device_file);
  pnh.getParam("device_file_2", led2_device_file);
  pnh.getParam("device_file_3", led3_device_file);

  ros::Subscriber sub = nh.subscribe("leds", 10, cb);

  ros::spin();
  exit(0);
}
