#!/usr/bin/env python3

#   Copyright 2024, Ricardo Rosales Martinez
 
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
 
#       http://www.apache.org/licenses/LICENSE-2.0
 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


__author__ = "Ricardo Rosales"
__version__ = "1.0"

import rclpy 
from rclpy.node import Node
from px4_msgs.msg import VehicleControlMode, VehicleOdometry, OffboardControlMode
from interaction_msgs.msg import ContactSetpoint 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import math 


class RectangleSetpoint(Node):

    def __init__(self):
        super().__init__('rectangle_setpoint')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Publishers and Subscribers
        self.control_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(ContactSetpoint, '/raw/trajectory_setpoint', 10)
        self.mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.flight_mode_callback, qos_profile)
        self.odometry_pub =self.create_subscription(VehicleOdometry,'/fmu/out/vehicle_odometry',self.odometry_callback,qos_profile)

        # Other parameters
        self.origin = np.array([0, 0, -1])
        self.x_length = 1.5 # [m]
        self.y_length = 0.8 # [m]
        self.z=np.array([-1.0,-1.0,-1.0,-1.0]) # [m]
        self.odometry=VehicleOdometry()
        self.threshold= 0.10 #tolerance to target position in m

        
        # Number of points between corners
        self.num_points = 2 # int 
        self.yaw=np.array([0,0,0,0]) # deg
        self.iter=0
        self.total_points=4*self.num_points

        # Yaw for each side of the rectangle
        self.offboard_mode = False
        # self.create_shape_with_points()
        # Set publish rate timer
        pub_rate = 30.0  # Hz. Set rate of > 2Hz for OffboardControlMode 
        # Adjust the timer to increase publishing speed of setpoints
        self.timer = self.create_timer(1/pub_rate, self.timer_callback)

        self.diagonal_trajectory = np.array([
            [1.0, 1.0, -5.0, 0],
            [1.0, 1.0, -5.0, 0],
            [1.0, 1.0, -5.0, 0],
            [1.0, 1.0, -5.0, 0]
        ])
    
    def flight_mode_callback(self,msg):
        self.offboard_mode=msg.flag_control_offboard_enabled
    
    def odometry_callback(self,msg):
        self.odometry=msg

    def calculate_distance(self,trajectory_points):
        x,y,z=self.odometry.position
        tx,ty,tz=trajectory_points[:3]
        # print("Test",trajectory_points[:3])
        distance=math.sqrt((tx-x)**2 + (ty-y)**2 + (tz-z)**2)
        # print("Distance",distance)

        return distance

        
    def timer_callback(self):
        # Set and publish control flags
        control_mode = OffboardControlMode()
        # Timestamp is automatically set inside PX4
        control_mode.timestamp = 0
        # First field that has a non-zero value (from top to bottom)
        # defines what valid estimate is required
        control_mode.position = True
        control_mode.velocity = False
        control_mode.acceleration  = False
        control_mode.attitude = False
        control_mode.body_rate = False
        control_mode.thrust_and_torque = False
        control_mode.direct_actuator = False
        self.control_pub.publish(control_mode)

        # Start sending setpoints if in offboard mode
        if self.offboard_mode:
            # Trajectory setpoint - NED local world frame
            contact_setpoint= ContactSetpoint()
            distance=self.calculate_distance(self.diagonal_trajectory[self.iter])
            if distance <= self.threshold:
                print("Reached")
                self.iter = (self.iter+1) % 4
            target_setpoints = self.diagonal_trajectory[self.iter]
            px=target_setpoints[0]
            py=target_setpoints[1]
            pz=target_setpoints[2]
            contact_setpoint.position=[px,py,pz]
            contact_setpoint.yaw = 0.0
            contact_setpoint.desired_force = -5.0 # Newtons
            contact_setpoint.contact=True
            self.setpoint_pub.publish(contact_setpoint)
        else:
            self.iter = 0
    
def main(args=None):
    # print('Hi from Offboard_programs.')
    rclpy.init(args=args)
    offboard_control=RectangleSetpoint()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
