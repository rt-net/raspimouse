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
 * https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/src/lightsensors.cpp
 * which is BSD 3-Clause "New" or "Revised" License.
 *
 * Copyright (c) 2017, Ryuichi Ueda
 * https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/LICENSE
 */

#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include "raspimouse_msgs/LightSensorValues.h"

int getFrequency(int old, ros::NodeHandle* n)
{
  int f;
  if (n->getParam("frequency", f) and f > 0)
    return f;

  return old;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "raspimouse_lightsensors_node");
  ros::NodeHandle n("~");

  ros::Publisher pub = n.advertise<raspimouse_msgs::LightSensorValues>("/lightsensors", 5);

  int freq = 10;

  ros::Rate loop_rate(freq);
  raspimouse_msgs::LightSensorValues msg;

  unsigned int c = 0;
  while (ros::ok())
  {
    if (c++ % freq == 0)
    {  // check the parapeter every 1[s]
      unsigned int old = freq;
      freq = getFrequency(freq, &n);
      if (old != freq)
      {
        loop_rate = ros::Rate(freq);
        ROS_INFO("Lightsensor frequency: %d", freq);
      }
    }

    std::ifstream ifs("/dev/rtlightsensor0");
    ifs >> msg.right_forward >> msg.right_side >> msg.left_side >> msg.left_forward;

    msg.sum_forward = msg.left_forward + msg.right_forward;
    msg.sum_all = msg.sum_forward + msg.left_side + msg.right_side;

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  exit(0);
}
