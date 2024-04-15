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
 * @file admittance_controller.hpp
 * @brief Header file for the admittance force controller node, used for aerial interactions 
 *
 * @author Ricardo Rosales Martinez
 */

#ifndef ADMITTANCE_CONTROLLER_
#define ADMITTANCE_CONTROLLER_


// ROS2 libraries 
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp" //leptrino wrench  f and torque*
#include "geometry_msgs/msg/wrench_stamped.hpp" //leptrino wrench  f and torque*
//px4 msgs/ odometry position used in for the manipulator 

class AdmittanceController : public rclcpp::Node {
    public: 
        AdmittanceController();

        //Destructor//
        ~AdmittanceController();
    private:
        void initROSNode();
        void declareParameters();
        void initUAVImpedance();
        void computeImpedance();

        void publishMessage();
        
        //// ROS variables ////
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub; // Later change to PX4 msg sub type
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub; //PX4 msg sub type
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif 