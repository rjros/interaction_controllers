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
 * @file direct_force_controller.cpp
 * @brief Direct force controller node, used for aerial interactions 
 *
 * @author Ricardo Rosales Martinez
 */


#include "direct_force_controller.hpp"
#include <cstdio>
#include <algorithm>
#include <iterator>


DirectForceController :: DirectForceController() : Node("Direct_wrench_control")
{
  
  //initialization ROS node
  RCLCPP_INFO(this->get_logger(), "%s\n","I am initializing the ROS node...");
  initNode();
  initController();

}


void DirectForceController :: initNode()
{
  int ms_time=10;//1000/rate_;
  wrench_sub_= this->create_subscription<geometry_msgs::msg::WrenchStamped>("leptrino_force_sensor/sensor_wrench", 1, 
  std::bind(&DirectForceController::WrenchSubCallback, this,_1));
  trajectory_sub_= this->create_subscription<interaction_msgs::msg::ContactSetpoint>("/raw/trajectory_setpoint", 10, 
  std::bind(&DirectForceController::ContactSubCallback, this,_1));
  publisher_= this->create_publisher<px4_msgs::msg::PlanarThrustSetpoint>("/fmu/in/planar_thrust_setpoint",10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(ms_time), std::bind(&DirectForceController::publishMessage, this));
  
}

//// ROS Functions ////

void DirectForceController :: publishMessage()
{
    
    if (contactRef_) {
        
        computeForceCmd();

    }
    else {

        forceCmd_={0,0,0};

    }

    auto message = std::make_unique<px4_msgs::msg::PlanarThrustSetpoint>();
    message->timestamp = 0; // Set timestamp to microseconds
 
    message->force[0]=forceCmd_[0]; // x body frame
    message->force[1]=forceCmd_[1]; // y  body frame
    message->force[2]=0.0f; // z body frame
    message->control_mode=contactRef_;

    publisher_->publish(std::move(message));


}

void DirectForceController :: ContactSubCallback(const interaction_msgs::msg::ContactSetpoint::SharedPtr msg )
{
  contactRef_=msg->contact;
  forceSp_=msg->desired_force;

}

void DirectForceController :: WrenchSubCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg )
{
  auto force= msg->wrench.force;
  auto torque= msg->wrench.torque;
  
  // Assign force and torque vectors directly to refForce_ and refTorque_
  forceRef_ = {force.x, force.y, force.z};
  torqueRef_ = {torque.x, torque.y, torque.z};
}

//// ROS Functions END ////


void DirectForceController::initController()
{
  // Get parameters from config file regarding the M, D and K matrices
  kp_=0.08;
  ki_=0.1;
  // forceSp_=-6; // in newtons 
  max_thrust_= 9.00; // max force per fans in newtons

  // RCLCPP_INFO(this->get_logger(), "%s\n","Initializing force controller...");// string followed by a newline

  efForce_=forceSp_/max_thrust_; //feedforward term

}
void DirectForceController::computeForceCmd()
{
    if (firstIteration_)
    {
      dt=0.01;
      firstIteration_=false;
      prevTime_=clock->now();
      eiForce_=0;
    } else {
        currentTime_ = clock->now();
        rclcpp::Duration duration = currentTime_ - prevTime_;
        prevTime_=clock->now();
        dt = duration.seconds(); //duration.seconds();
    }
    //prevTime_= currentTime_;
    eiForce_+= ki_*epForce_*dt;
    if (abs(eiForce_)>1)
    {
      eiForce_=0;
    }
    RCLCPP_INFO(this->get_logger(), "Force X %f Force Y %f Force Z %f", dt, eiForce_, forceRef_[2]);
    epForce_=forceSp_-forceRef_[2]; //force in the z axis
    forceCmd_[0]= -efForce_ - kp_ * (epForce_) - eiForce_;
    forceCmd_[1]= 0.0;
    forceCmd_[2]= 0.0;

}

DirectForceController :: ~DirectForceController()
{
 RCLCPP_INFO(this->get_logger(), "Shutting down node.");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  std::shared_ptr<DirectForceController> node = std::make_shared<DirectForceController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
