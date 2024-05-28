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

#ifndef DIRECT_FORCE_CONTROLLER_
#define DIRECT_FORCE_CONTROLLER_


// ROS2 libraries 
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp> //leptrino wrench  f and torque*

#include <px4_msgs/msg/planar_thrust_setpoint.hpp>
#include <interaction_msgs/msg/contact_setpoint.hpp>

#include <Eigen/Dense> // msg type to be published with correction
using std::placeholders::_1; // Place holder for ros subscriptions


class DirectForceController : public rclcpp::Node {
    
    public: 
        //// CONSTRUCTOR ////
        /*constructor of the class*/
        DirectForceController();
    
        //Destructor//
        ~DirectForceController();
    private:
        
        void initNode();
        // void declareParameters();
        void initController();
        void computeForceCmd();

        //ROS Related callbacks
        void ContactSubCallback(const interaction_msgs::msg::ContactSetpoint::SharedPtr msg);
        void WrenchSubCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
        void publishMessage();
        
        //// ROS variables ////
        rclcpp::Subscription<interaction_msgs::msg::ContactSetpoint>::SharedPtr trajectory_sub_; // Later change to PX4 msg sub type
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_; //PX4 msg sub type
        rclcpp::Publisher<px4_msgs::msg::PlanarThrustSetpoint>::SharedPtr publisher_;

  
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
        rclcpp::Time prevTime_;
        rclcpp::Time currentTime_;
        bool firstIteration_{true};
        double dt{0};

        //Internal variables

        std::vector<double> forceCmd_ = std::vector<double>(3,0);
        std::vector<double> torqueCmd_ = std::vector<double>(3,0);

        std::vector<double>quaternionRef_ = std::vector<double>(4,0);
        std::vector<double>quaternionSp_ = std::vector<double>(4,0);

        std::vector<double> forceRef_ = std::vector<double>(3,0);
        std::vector<double>torqueRef_ = std::vector<double>(3,0);
        std::vector<double> eulerRef_ = std::vector<double>(3,0);
        

        // Using a PI controller for the force
        double kp_{0};
        double ki_{0};
        double kyp_{0};

        double dt_={0};

        double epForce_{0};
        double eiForce_{0};
        double efForce_{0};
        double yawSp_{0}; 
        double epYaw_{0};

        double max_thrust_={0};


        int rate_;
        // Force Control internal variables 
        double forceSp_;
        double yawRef_;
        bool contactRef_{false};


};

#endif 