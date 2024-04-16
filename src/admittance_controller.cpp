/*
 * Copyright 2024, Ricardo Rosales Martinez
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

/**
 * @file admittance_controller.cpp
 * @brief Admittance force controller node, used for aerial interactions 
 *
 * @author Ricardo Rosales Martinez
 */


#include "admittance_controller.hpp"
#include <cstdio>
#include <algorithm>
#include <iterator>
using std::placeholders::_1; // Place holder for ros subscriptions


AdmittanceController :: AdmittanceController() : Node("admittance_wrench_control") 
{
  // int ms_time=10;//1000/rate_;
  state_sub_= this->create_subscription<geometry_msgs::msg::PoseStamped>("px4_odometry_topic", 1, 
  std::bind(&AdmittanceController::StateSubCallback, this,_1));

  wrench_sub_= this->create_subscription<geometry_msgs::msg::WrenchStamped>("leptrino_wrench_topic", 1, 
  std::bind(&AdmittanceController::WrenchSubCallback, this,_1));
  // publisher_= this->create_publisher<geometry_msgs::msg::PoseStamped>("force_publisher",10);
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(ms_time), std::bind(&AdmittanceController::publishMessage, this));
}

// msg to be sent to px4 or position publisher...

//// ROS Functions ////

void AdmittanceController :: publishMessage(){
    auto message = std::make_unique<geometry_msgs::msg::PoseStamped>();
    message->header.stamp = this->now(); // Set timestamp
    message->pose.position.x = 0.0;      // Set position (example values)
    message->pose.position.y = 0.0;
    message->pose.position.z = 0.0;
}

void AdmittanceController :: StateSubCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg )
{
  auto position =msg->pose.position;
  refPosition_={position.x,position.y,position.z};

  // std::copy(std::begin(position),std::end(position),std::begin(refPosition_));

}

void AdmittanceController :: WrenchSubCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg )
{
  auto force= msg->wrench.force;
  auto torque= msg->wrench.torque;

  // Assign force and torque vectors directly to refForce_ and refTorque_
  refForce_ = {force.x, force.y, force.z};
  refTorque_ = {torque.x, torque.y, torque.z};

  // std::copy(std::begin(force),std::end(force),std::begin(refForce_));
  // std::copy(std::begin(torque),std::end(torque),std::begin(refTorque_));

}

//// ROS Functions END ////


void AdmittanceController::initUAVImpedance(){

}
void AdmittanceController::computeImpedance(){

}





int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  std::shared_ptr<AdmittanceController> node = std::make_shared<AdmittanceController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
