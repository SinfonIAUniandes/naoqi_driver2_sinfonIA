/*
 * Copyright 2015 Aldebaran
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
*/

/*
 * LOCAL includes
 */
#include "teleop.hpp"

#include <algorithm>
#include <cmath>


namespace naoqi
{
namespace subscriber
{

TeleopSubscriber::TeleopSubscriber( const std::string& name, const std::string& cmd_vel_topic, const std::string& joint_trajectory_topic, const qi::SessionPtr& session ):
  cmd_vel_topic_(cmd_vel_topic),
  joint_trajectory_topic_(joint_trajectory_topic),
  BaseSubscriber( name, cmd_vel_topic, session ),
  p_motion_( session->service("ALMotion").value() )
{}

void TeleopSubscriber::reset( rclcpp::Node* node )
{
  sub_cmd_vel_ = node->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_,
    10,
    std::bind(&TeleopSubscriber::cmd_vel_callback, this, std::placeholders::_1));

  sub_joint_trajectory_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    joint_trajectory_topic_,
    10,
    std::bind(&TeleopSubscriber::joint_trajectory_callback, this, std::placeholders::_1));

  is_initialized_ = true;
}

void TeleopSubscriber::cmd_vel_callback( const geometry_msgs::msg::Twist::SharedPtr twist_msg )
{
  // no need to check for max velocity since motion clamps the velocities internally
  const float& vel_x = twist_msg->linear.x;
  const float& vel_y = twist_msg->linear.y;
  const float& vel_th = twist_msg->angular.z;

  std::cout << "going to move x: " << vel_x << " y: " << vel_y << " th: " << vel_th << std::endl;
  p_motion_.async<void>("move", vel_x, vel_y, vel_th );
}

void TeleopSubscriber::joint_trajectory_callback( const trajectory_msgs::msg::JointTrajectory::SharedPtr trajectory_msg )
{
  if (trajectory_msg->points.empty()) {
    std::cerr << "Received empty joint trajectory" << std::endl;
    return;
  }

  const auto& point = trajectory_msg->points.front();
  if (trajectory_msg->joint_names.empty() || point.positions.size() != trajectory_msg->joint_names.size()) {
    std::cerr << "Joint trajectory command must include matching joint_names and first-point positions" << std::endl;
    return;
  }

  float speed = 0.2f;
  if (!point.velocities.empty()) {
    speed = std::clamp(static_cast<float>(std::abs(point.velocities.front())), 0.0f, 1.0f);
  }

  p_motion_.async<void>("setAngles", trajectory_msg->joint_names, point.positions, speed);
}

} //publisher
} // naoqi
