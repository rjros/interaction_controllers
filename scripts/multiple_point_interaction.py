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
from px4_msgs.msg import VehicleControlMode, VehicleOdometry, OffboardControlMode, TrajectorySetpoint, ForceControlMode
from interaction_msgs.msg import ContactSetpoint 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import math 
import time
# from tf_transformations import  euler_from_quaternion





class WallSetpoint(Node):

    def __init__(self):
        super().__init__('multiple_point_interaction')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Publishers and Subscribers
        self.control_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.contact_pub = self.create_publisher(ContactSetpoint, '/raw/trajectory_setpoint', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.flight_mode_callback, qos_profile)
        
        self.mode_sub = self.create_subscription(ForceControlMode, '/fmu/out/force_control_mode', self.force_mode_callback, qos_profile)

        self.odometry_pub =self.create_subscription(VehicleOdometry,'/fmu/out/vehicle_odometry',self.odometry_callback,qos_profile)


        # Trajectory setpoints 
        self.odometry=VehicleOdometry()
        self.offboard_mode = False
        self.threshold= 0.10 #tolerance to target position in m
        self.yaw_sp= 0.0 # wall orientation
        # Contact Variables
        self.force1 = -3.0 #desired force in Newtons
        self.force2 = -5.0 #desired force in Newtons
        self.force3 = -6.0 #desired force in Newtons
        self.force3 = -8.0 #desired force in Newtons


        self.iter=0
        self.start_time=0
        self.current_time=0
        self.contact_time=10 # s
        self.fly_time=1 # s
        self.wait= False
        self.wait_time=0
        self.reached_flag=False

        # Adjust the timer to increase publishing speed of setpoints
        pub_rate = 100  # Hz. Set rate of > 2Hz for OffboardControlMode 
        self.timer = self.create_timer(1/pub_rate, self.timer_callback)

        # [x,y,z,time,contact] 6
        self.wall_setpoints = np.array([
            [3.8, 0.0, -1.6, 0,self.fly_time,0.0],
            [4.1, 0.0, -1.6, 0,0,0],
            [4.1, 0.0, -1.6, 1,self.contact_time,self.force1],
            [3.8, 0.0, -1.6, 0,self.fly_time,0.0],
            [3.8, 0.5, -1.6, 0,self.fly_time,0.0],
            [4.1, 0.5, -1.6, 0,0,0],
            [4.1, 0.5, -1.6, 1,self.contact_time,self.force2],
            [3.8, 0.5, -1.6, 0,self.fly_time,0.0],
            [3.8, 1.0, -1.6, 0,self.fly_time,0.0],
            [4.1, 1.0, -1.6, 0,0,0],
            [4.1, 1.0, -1.6, 1,self.contact_time,self.force3],
            [3.8, 1.0, -1.6, 0,self.fly_time,0.0],
            [3.8, 1.5, -1.6, 0,self.fly_time,0.0],
            [4.1, 1.5, -1.6, 0,0,0],
            [4.1, 1.5, -1.6, 1,self.contact_time,self.force1],
            [3.8, 1.5, -1.6, 0,self.fly_time,0.0]
        ])
    
    def flight_mode_callback(self,msg):
        self.offboard_mode=msg.flag_control_offboard_enabled
    
    def force_mode_callback(self,msg):
        self.force_mode=msg.force_ctrl
        print(self.force_mode)
    
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

        contact_setpoint = ContactSetpoint()  
        contact_setpoint.desired_force = -3.0
        contact_setpoint.contact=bool(self.force_mode)
        self.contact_pub.publish(contact_setpoint)


    
def main(args=None):
    # print('Hi from Offboard_programs.')
    rclpy.init(args=args)
    offboard_control=WallSetpoint()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
