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
from px4_msgs.msg import VehicleControlMode, VehicleOdometry, OffboardControlMode, TrajectorySetpoint
from interaction_msgs.msg import ContactSetpoint 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import math 
import time



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
        self.odometry_pub =self.create_subscription(VehicleOdometry,'/fmu/out/vehicle_odometry',self.odometry_callback,qos_profile)


        # Trajectory setpoints 
        self.odometry=VehicleOdometry()
        self.offboard_mode = False
        self.threshold= 0.15 #tolerance to target position in m
        self.yaw_sp= 0.0 # wall orientation
        # Contact Variables
        self.force1 = -3.0 #desired force in Newtons
        self.force2 = -3.0 #desired force in Newtons
        self.force3 = -3.0 #desired force in Newtons


        self.iter=0
        self.start_time=0
        self.current_time=0
        self.contact_time=2 # s
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
            [4.56, 0.0, -1.6, 1,self.contact_time,self.force1],
            [4.56, 0.5, -1.6, 1,self.contact_time,self.force2],
            [4.56, 1.0, -1.6, 1,self.contact_time,self.force3],
            [4.56, 1.5, -1.6, 1,self.contact_time,self.force1],
            [3.8, 1.5, -1.6, 0,self.fly_time,0.0]
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
        # control_mode.thrust_and_torque = False
        # control_mode.direct_actuator = False
        self.control_pub.publish(control_mode)
        contact_setpoint = ContactSetpoint()
        trajectory_setpoint = TrajectorySetpoint()

        # Start sending setpoints if in offboard mode
        if self.offboard_mode:
            # Trajectory setpoint - NED local world frame
            
            if(self.wait):
                self.current_time=time.time()
                if(self.current_time-self.start_time>=self.wait_time):
                    self.wait=False
                    self.reached_flag=False
                    self.iter = (self.iter+1) % 6

            setpoints = self.wall_setpoints[self.iter]
            px = setpoints[0]
            py = setpoints[1]
            pz = setpoints[2]
            trajectory_setpoint.position = [px, py, pz]
            trajectory_setpoint.yaw = self.yaw_sp
            
            contact_setpoint.desired_force = setpoints[5]

            self.wait_time=setpoints[4]
            distance=self.calculate_distance(self.wall_setpoints[self.iter])
            if (distance <= self.threshold):
                self.reached_flag = True 
                print("Reached")

            if (self.reached_flag and not self.wait):
                self.wait=True
                self.start_time=time.time()
            if (self.reached_flag):
                contact_setpoint.contact=bool(setpoints[3]>0)
            print(self.iter)
            contact_setpoint.contact=bool(setpoints[3]>0)
            self.setpoint_pub.publish(trajectory_setpoint)
            self.contact_pub.publish(contact_setpoint)
        else:
            self.iter = 0
            self.wait=False
            contact_setpoint.contact=False



    
def main(args=None):
    # print('Hi from Offboard_programs.')
    rclpy.init(args=args)
    offboard_control=WallSetpoint()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
