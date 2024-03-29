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
 * https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/src/motors.cpp
 * which is BSD 3-Clause "New" or "Revised" License.
 *
 * Copyright (c) 2017, Ryuichi Ueda
 * https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/LICENSE
 */

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include <std_srvs/Trigger.h>
#include "raspimouse_control/raspimouse_hardware.hpp"

bool is_on = false;
std::string motor_sw_device_file = "/dev/rtmotoren0";
bool callbackOn(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
bool callbackOff(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);

bool setPower(bool onoff)
{
  std::ofstream ofs(motor_sw_device_file);
  if (not ofs.is_open())
    return false;

  ofs << (onoff ? '1' : '0') << std::endl;
  is_on = onoff;
  return true;
}

bool callbackOn(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  if (not setPower(true))
  {
    return false;
  }

  response.message = "ON";
  response.success = true;
  return true;
}

bool callbackOff(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  if (not setPower(false))
  {
    return false;
  }

  response.message = "OFF";
  response.success = true;
  return true;
}

void sigintHandler(int sig)
{
  setPower(false);
  ros::shutdown();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, ros::this_node::getName(), ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("device_file_motor_sw", motor_sw_device_file);

  std::string onoff = "off";
  if (argc > 1)
    onoff = argv[1];
  setPower(onoff == "on");

  signal(SIGINT, sigintHandler);

  ros::ServiceServer srv_on = nh.advertiseService("motor_on", callbackOn);
  ros::ServiceServer srv_off = nh.advertiseService("motor_off", callbackOff);

  RaspberryPiMouseHW raspimouse(nh, pnh);
  controller_manager::ControllerManager cm(&raspimouse, nh);

  ros::Rate loop_rate(30);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time t = ros::Time::now();
  ros::Duration d = ros::Time::now() - t;

  while (ros::ok())
  {
    d = ros::Time::now() - t;
    t = ros::Time::now();
    // ROS_INFO("read %u %u ", d.nsec, t.nsec);
    raspimouse.read(d);
    cm.update(t, d);
    raspimouse.write();
    loop_rate.sleep();
  }

  spinner.stop();

  return 0;
}
