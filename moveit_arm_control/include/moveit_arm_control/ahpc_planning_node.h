/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Boston Cleek
   Desc: A simple demo node running ompl constrained planning capabilities for planning and execution
*/
#ifndef CONSTRAINED_PLANNING_NODE_H_
#define CONSTRAINED_PLANNING_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_driver_status.h"
#include "shared_message/msg/robot_driver_status.hpp"
#include "shared_message/srv/move_to_location_position.hpp"
class AHPCPlanning
{
public:
  AHPCPlanning(const rclcpp::NodeOptions& options);
  ~AHPCPlanning();
  void doTask();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
private:

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_;
  //topic
  rclcpp::Subscription<shared_message::msg::RobotDriverStatus>::SharedPtr robot_driver_status_sub_; 
  //service
  rclcpp::Service<shared_message::srv::MoveToLocationPosition>::SharedPtr move_to_location_pos_srv_;
  robot_driver_status driver_status_;      /* 机器人驱动状态 */ 
  bool single_task_update_status_;
  int loc_type_;
  std::vector<float> vec_loc_pose_;
  void robotDriverStatusHandle(const shared_message::msg::RobotDriverStatus::ConstSharedPtr driver_status);
  void moveToLocationPosition(const std::shared_ptr<shared_message::srv::MoveToLocationPosition::Request> request,
                              std::shared_ptr<shared_message::srv::MoveToLocationPosition::Response> response);
  void baseToTcp(geometry_msgs::msg::Pose& cur_pose,std::vector<float> delta_pose,geometry_msgs::msg::Pose& tar_pose);
};

#endif //CONSTRAINED_PLANNING_NODE_H_