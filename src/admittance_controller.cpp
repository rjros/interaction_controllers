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
// #include <interaction_controllers/msg/contact_setpoint.h>


AdmittanceController :: AdmittanceController() : Node("admittance_wrench_control") 
{
  
  //initialization ROS node
  RCLCPP_INFO(this->get_logger(), "%s\n","I am initializing the ROS node...");
  initNode();
  initAdmittance();

}


void AdmittanceController :: initNode()
{
  int ms_time=10;//1000/rate_;
  trajectory_sub_= this->create_subscription<px4_msgs::msg::TrajectorySetpoint>("/raw/trajectory_setpoint", 10, 
  std::bind(&AdmittanceController::TrajectorySubCallback, this,_1));
  wrench_sub_= this->create_subscription<geometry_msgs::msg::WrenchStamped>("leptrino_force_sensor/sensor_wrench", 1, 
  std::bind(&AdmittanceController::WrenchSubCallback, this,_1));
  publisher_= this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint",10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(ms_time), std::bind(&AdmittanceController::publishMessage, this));
  
}

//// ROS Functions ////

void AdmittanceController :: publishMessage()
{
  computeAdmittance();


    auto message = std::make_unique<px4_msgs::msg::TrajectorySetpoint>();
    message->timestamp = 0; // Set timestamp to microseconds
    
    // Final position should be the XD*=XD + f/k
    //pushing only to the front X axis of the UAV, convert to Body Frame
    // message->pose.position.x = std::round(positionSp_[0]*1000)/1000;   // Set position (example values)
    // message->pose.position.y = positionSp_[1];
    // message->pose.position.z = positionSp_[2];

    message->position[0]=positionSp_[0];
    message->position[1]=positionSp_[1];
    message->position[2]=positionSp_[2];
    message->yaw=yawSp_;
    
    publisher_->publish(std::move(message));

}

void AdmittanceController :: TrajectorySubCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg )
{
  auto position =msg->position;
  auto yaw = msg->yaw;
  trajectoryRef_={position[0],position[1],position[2],yaw};
  yawSp_ = yaw;

  // RCLCPP_INFO(this->get_logger(), "Force X %f Force Y %f Force Z %f", forceRef_[0], forceRef_[1], forceRef_[2]);

  // std::copy(std::begin(position),std::end(position),std::begin(refPosition_));

}

void AdmittanceController :: WrenchSubCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg )
{
  auto force= msg->wrench.force;
  auto torque= msg->wrench.torque;

  // Assign force and torque vectors directly to refForce_ and refTorque_
  forceRef_ = {force.x, force.y, force.z};
  torqueRef_ = {torque.x, torque.y, torque.z};
}

//// ROS Functions END ////


void AdmittanceController::initAdmittance()
{
  // Get parameters from config file regarding the M, D and K matrices
  K_=0.1;
  forceSp_=-5;

  RCLCPP_INFO(this->get_logger(), "%s\n","Initializing admittance controller...");// string followed by a newline

}
void AdmittanceController::computeAdmittance()
{
  RCLCPP_INFO(this->get_logger(),"%s\n","Computing admittance controller...");
  // RCLCPP_INFO(this->get_logger(), "Force X %f Force Y %f Force Z %f", forceRef_[0], forceRef_[1], forceRef_[2]);

  //// Force in the axis of the manipulator ////
  /// Rotate wrt to the Yaw (B)
  positionSp_[0] = trajectoryRef_[0] + (forceRef_[2]-forceSp_)/K_;
  positionSp_[1] = trajectoryRef_[1] ;//positionRef_[1]; 
  positionSp_[2] = trajectoryRef_[2] ;//positionRef_[2];

}

AdmittanceController :: ~AdmittanceController()
{
 RCLCPP_INFO(this->get_logger(), "Shutting down node.");
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  std::shared_ptr<AdmittanceController> node = std::make_shared<AdmittanceController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
