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
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp> //leptrino wrench  f and torque*
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <interaction_msgs/msg/contact_setpoint.hpp>

#include <Eigen/Dense> // msg type to be published with correction
using std::placeholders::_1; // Place holder for ros subscriptions


class AdmittanceController : public rclcpp::Node {
    
    public: 
        //// CONSTRUCTOR ////
        /*constructor of the class*/
        AdmittanceController();
    
        //Destructor//
        ~AdmittanceController();
    private:
        
        void initNode();
        // void declareParameters();
        void initAdmittance();
        void computeAdmittance();

        //ROS Related callbacks
        void TrajectorySubCallback(const interaction_msgs::msg::ContactSetpoint::SharedPtr msg);
        void WrenchSubCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
        void publishMessage();
        
        //// ROS variables ////
        rclcpp::Subscription<interaction_msgs::msg::ContactSetpoint>::SharedPtr trajectory_sub_; // Later change to PX4 msg sub type
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_; //PX4 msg sub type
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        //Internal variables
    
        std::vector<double> trajectoryRef_ = std::vector<double>(4,0);
        std::vector<double> positionSp_ = std::vector<double>(3,0);
        float yawSp_; 


        std::vector<double>quaternionRef_ = std::vector<double>(4,0);
        std::vector<double>quaternionSp_ = std::vector<double>(4,0);

        std::vector<double> forceRef_ = std::vector<double>(3,0);
        std::vector<double>torqueRef_ = std::vector<double>(3,0);
        std::vector<double> eulerRef_ = std::vector<double>(3,0);

        // Inertial (M), Damping (D) and Stiffness (K) matrices

        double K_{0};


        int rate_;
        // Force Control internal varaibles 
        double forceSp_;
        bool contactRef_{false};


};

#endif 