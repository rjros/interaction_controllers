# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
###################################################################################

__author__ = "Ricardo Rosales"
__version__ = "1.0"

import rclpy 
from rclpy.node import Node
from rclpy.time import Time
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleControlMode, VehicleOdometry,ThrustVectoringSetpoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import time
class TiltingFrameRotation(Node):

    def __init__(self):
        super().__init__('tilting_frame_rotation')
        qos_profile=QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
            depth=1
        )
    
    # Publishers and Subscribers
        self.control_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.thrust_vec_setpoint_pub =self.create_publisher(ThrustVectoringSetpoint,'/fmu/in/thrust_vectoring_setpoint',1)
        self.mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.flight_mode_callback, qos_profile)
        self.odometry_sub =self.create_subscription(VehicleOdometry,'/fmu/out/vehicle_odometry',self.odometry_callback,qos_profile)

    
    # Parameter origin and distance 
        self.origin_x = 0 # [m]
        self.origin_y = 0 # [m]
        self.origin_z = 3 # [m] use z axis up as positive
        self.length = 1 # [m]
        self.num_points = 5 # pointes per line
        self.start_time=0
        self.current_time=0
    # Initial Line to be Rotated
        self.x_values =  np.linspace(0,self.length,self.num_points)
        self.y_values = np.zeros(self.num_points)
        self.z_values = np.zeros(self.num_points)

        self.tilt_angles = np.ones(self.num_points)
        self.trajectory_setpoints = np.empty((0,4),float) 
    
    # Angle Parameters
        self.total_rotation= 360 # Span of rotation in degrees
        self.angle_div = 45 # angle division of desired rotation
        self.start_angle = 270# Initial orientation to begin the trajectory
        self.first_half_angle=90 ## end of first trajectory
        self.second_half_angle= 270 ## end pos for second half 
        self.angle_range = range(0,self.total_rotation+1,self.angle_div)
        
    # Index Positions 
        self.start_idx= int(self.start_angle/self.angle_div*self.num_points*2) # Which line and angle will be used in the cw trajectory
        self.iter=self.start_idx 
        self.end_first_idx =  int((self.first_half_angle+self.angle_div)/self.angle_div*self.num_points*2) 
        self.end_second_idx = int((self.second_half_angle+self.angle_div) /self.angle_div*self.num_points*2) 

    
    # Flags for trajectories
        self.first_half=False # run from -90 to 90
        self.second_half=False # 90 to -90 
        self.wait= False

         
    # Calculate trajectory to follow
        self.calculate_trajectory()
        self.total_line=self.trajectory_setpoints.shape[0]
        print(self.iter,self.total_line)


    
    #Ros Parameters
        self.odometry=VehicleOdometry()
    

     # Set publish rate timer
        pub_rate = 30.0  # Hz. Set rate of > 2Hz for OffboardControlMode 
        self.offboard_mode = False

     # Adjust the timer to increase publishing speed of setpoints
        self.timer = self.create_timer(1/pub_rate, self.timer_callback)

    def flight_mode_callback(self,msg):
        self.offboard_mode=msg.flag_control_offboard_enabled
    
    def odometry_callback(self,msg):
        self.odometry=msg

    def rotate_points(self,x,z,angle_deg):
        angle_radians = np.radians(angle_deg)
        x_rotated = x * np.cos(angle_radians) - z * np.sin(angle_radians) + self.origin_x
        z_rotated = x * np.sin(angle_radians) + z * np.cos(angle_radians) + self.origin_z

        return x_rotated, z_rotated

    def calculate_trajectory(self):
        for angle in self.angle_range:
        # Rotate the points
            rotated_x, rotated_z= zip(*[self.rotate_points(x, y, angle) for x, y in zip(self.x_values, self.z_values)])
            # Save the rotated points to the array
            rotated_points = np.column_stack((np.round(rotated_x,3),self.y_values,np.round(rotated_z,3),self.tilt_angles*angle))
            ## return to 
            reverse_rotated_points=rotated_points[::-1]
            combined_points=np.concatenate((rotated_points,reverse_rotated_points),axis=0)
            self.trajectory_setpoints = np.vstack((self.trajectory_setpoints, combined_points))
    
    def timer_callback(self):
        # Set and publish control flags
        control_mode = OffboardControlMode()
        thrust_vect_set= ThrustVectoringSetpoint()
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
            setpoint_trajectory =TrajectorySetpoint()
            # Timestamp is automatically set inside PX4
            setpoint_trajectory.timestamp = 0
            # Check time after reaching setpoints 
            if(self.wait):
                self.current_time=time.time()
                if(self.current_time-self.start_time>=3):
                    self.wait=False
                else:
                    self.iter=self.iter-1

            if (not self.first_half):

                ## Crosses the 0, 360 angle
                if(self.iter>self.total_line-1):
                    self.iter=self.num_points*2

                target_setpoints= self.trajectory_setpoints[self.iter]
                self.iter = self.iter +1
                setpoint_trajectory.yaw = 0.0 
                thrust_vect_set.servo_angle[0]= (target_setpoints[3] + 90) % 360 - 90
                self.first_half=(self.iter==self.end_first_idx)

            else:
                target_setpoints= self.trajectory_setpoints[self.iter]
                self.iter = self.iter + 1
                setpoint_trajectory.yaw = np.radians(180)
                thrust_vect_set.servo_angle[0]= 180-target_setpoints[3]
                self.second_half= (self.iter==self.end_second_idx)

            if(self.first_half and self.second_half):
                self.first_half=False
                self.second_half= False
            px= target_setpoints[0]
            py = target_setpoints [1]
            pz = -target_setpoints [2]
            setpoint_trajectory.position = [px, py, pz] # [x, y, z] in meters
            #setpoint_traj.velocity # in m/s
            #setpoint_traj.acceleration # in m/s^2
            #setpoint_traj.jerk # m/s^3 (for logging only)
            #setpoint_traj.yawspeed = 0.0 # in rad/s
            # Publish
            #Wait when on start or end of line
            self.setpoint_pub.publish(setpoint_trajectory)
            self.thrust_vec_setpoint_pub.publish(thrust_vect_set)
            if(self.iter%self.num_points==0 and not self.wait):
              self.start_time=time.time()
              self.wait=True
            print(thrust_vect_set.servo_angle[0])
        else:
            self.iter = self.start_idx
            self.first_half = False
            self.second_half = False
            # print(self.iter)

def main(args=None):
    # print('Hi from Offboard_programs.')
    rclpy.init(args=args)
    offboard_control=TiltingFrameRotation()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    
    

    





