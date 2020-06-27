/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "waypoint_publisher_aisle");
  ros::NodeHandle nh("");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Started waypoint_publisher.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  double delay = 0.5;

  double x_start;
  double y_start;
  double x_stop;
  double y_stop;
  int steps; 
  double altitude;
  double orientation;

  if (args.size() == 8) {
     x_start = std::stof(args.at(1));
    y_start = std::stof(args.at(2));
     x_stop = std::stof(args.at(3));
     y_stop = std::stof(args.at(4));
    steps = std::stof(args.at(5));
     altitude = std::stof(args.at(6));
     orientation = std::stof(args.at(7));
  }
  else{
    ROS_ERROR("Usage: waypoint_publisher_aisle <x_start> <y_start> <x_start> <y_start> <steps> <altitude> <orientation> \n");
    return -1;
  }

  const float DEG_2_RAD = M_PI / 180.0;

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  for (int i = 0; i<=steps; i++ )
  {  
    double wp_x = x_start + i*((x_stop - x_start)/steps);
    double wp_y = y_start + i*((y_stop - y_start)/steps);
    double wp_z = altitude;

    Eigen::Vector3d desired_position(wp_x, wp_y, wp_z);

    double desired_yaw = orientation * DEG_2_RAD;

    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
        desired_yaw, &trajectory_msg);

    // Wait for some time to create the ros publisher.
    // ros::Duration(delay).sleep();

    while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
      ROS_INFO("There is no subscriber available, trying again in 1 second.");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("Publishing waypoint on namespace %s: [x: %f, y: %f, z: %f, yaw: %f].",
             nh.getNamespace().c_str(),
             desired_position.x(),
             desired_position.y(),
             desired_position.y(),
             desired_yaw);
    trajectory_pub.publish(trajectory_msg);

    ros::Duration(delay).sleep();

  }

  return 0;
}
