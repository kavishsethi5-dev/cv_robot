#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import math
import time
import numpy as np
from ros2_aruco_interfaces.msg import ArucoMarkers ###
from scipy.spatial.transform import Rotation as R #required to multiply 4 element arrays

class diff_PID(Node):

    def __init__(self):
        super().__init__('diff_PID')
        self.create_subscription(ArucoMarkers, '/aruco_markers', self.odom_callback, 10)
        self.subscriber_target_=self.create_subscription( Float64MultiArray, '/target', self.target_callback, 10)
        self.publishers_cmd_vel_=self.create_publisher(Twist, '/cmd_vel', 10)
        self.coord_publisher_ = self.create_publisher(Pose, 'camera_transformed_coord', 10) #publish trans coord to seq node 
        #Im using (geometry_msgs.msg pose) message since that was what we were using before for ground pose

        self.target_x=0.0 #empty value 
        self.target_y=0.0

        # Defined once in __init__
        self.kp = 1.2
        self.ki= 0.01
        self.kd = 0.05
        #Decided to add angular kp,ki,kd values also (will add reason later )
        self.kp_a = 4.1 #was 4 earlier
        self.ki_a = 0.01
        self.kd_a = 0.05 #assuming similar values
        
        ####Initializing variables####
        self.last_error = 0.0
        self.error_sum = 0.0
         #  Extract Current Position (X, Y)
        self.x_current = 0.0
        self.y_current = 0.0
        self.z_current =0.0

        # 2. Extract and Calculate Current Heading (Theta)
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 0.0
        # similar variables for angular
        self.last_error_a = 0.0
        self.error_sum_a = 0.0
        

        


        #for now setting dt as 0.01, later I can see it in gazebo physics
        self.target_x = None
        self.target_y = None

        self.last_time = None   #


        #create timer
        time_period= 0.0357 # /camera/image hz ag 28hz=0.0357
         # seconds (0.01 itt was earlier)
        self.timer = self.create_timer( time_period , self.PID_callback)

    def PID_callback(self):

        # Safety
        if self.target_x is None:
            return 
        if self.last_time is None:
            self.last_time = self.get_clock().now()
            return

        siny_cosp = 2 * (self.qw * self.qz + self.qx * self.qy)
        cosy_cosp = 1 - 2 * (self.qy * self.qy + self.qz * self.qz)
        theta_current = math.atan2(siny_cosp, cosy_cosp)

        # Math Formula 1: Distance Error 
        distance_error = math.sqrt((self.target_x - self.x_current)**2 + (self.target_y - self.y_current)**2)
        self.current_error = distance_error
        
        # Math Formula 2: Desired Heading Angle (Alpha)

        alpha = math.atan2(self.target_y - self.y_current, self.target_x - self.x_current)
        
        # Math Formula 3: Angular Error
        angle_error = alpha - theta_current
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        #Normalize angle_error to stay within [-pi, pi] eg 90 vs -270
        # defining self current error first
        self.current_error_a = angle_error



        self.get_logger().info(
        f"CHECK: distance_error={distance_error:.4f} | threshold=0.30 | "
        f"will_stop={distance_error < 0.30}"
        )

        if distance_error < 0.30:
            self.get_logger().warn(f"STOPPING at distance_error={distance_error:.4f}")
            fmsg = Twist()
            self.publishers_cmd_vel_.publish(fmsg)
            return


        #self.get_logger().info(
            #f"Target: ({self.target_x}, {self.target_y}) | "
            #f"Dist : {distance_error:.2f}m | "
            #f"Ang: {math.degrees(angle_error):.1f}°")
        #tp
        now = self.get_clock().now()
        time_step = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now   # update for next call
    
        if time_step <= 0.01:
            return 

        #now using the calc to get PID
        P = self.kp * self.current_error
        I = self.ki * self.error_sum
        self.error_sum += self.current_error * time_step
        error_change = (self.current_error - self.last_error) / time_step
        D = self.kd * error_change

        #PID calc for angular
        P_a = self.kp_a * self.current_error_a
        self.error_sum_a += self.current_error_a * time_step
        I_a = self.ki_a * self.error_sum_a
        error_change_a = (self.current_error_a - self.last_error_a) / time_step
        D_a = self.kd_a * error_change_a


        #updating the last value
        self.last_error = self.current_error
        self.last_error_a = self.current_error_a
        


        #final
        pid_dist = min(P + I + D, 1.0)  
        pid_angle = max(-2.0, min(2.0, P_a + I_a + D_a))


        if distance_error < 0.30:  #this was done to stop it from circling at the end
            fmsg = Twist()   # zero velocity
            self.publishers_cmd_vel_.publish(fmsg)
            return           # exits PID_callback right here
        
        
        fmsg = Twist()
        heading_factor = max(0.0, math.cos(angle_error)) # cos angle error or cos pid_angle?
        # this is to stop the spinning (also tuning ka_a)
        fmsg.linear.x =  pid_dist* heading_factor  # drives at ~0 at 163°, full speed when aligned
        fmsg.angular.z = pid_angle


        #publish this message to cmd_vel
        self.publishers_cmd_vel_.publish(fmsg)
        #reset (resetting the error sum and last error for next iteration)
        self.last_time = self.get_clock().now() 
      
    def odom_callback(self, msg):
        for i in range(len(msg.marker_ids)):
            marker_ids=msg.marker_ids[i]
            pose=msg.poses[i]
            temp_x = pose.position.x  # x=msg.poses.position.x
            temp_y = pose.position.y
            temp_z = pose.position.z
        
            qx = pose.orientation.x 
            qy = pose.orientation.y 
            qz = pose.orientation.z
            qw = pose.orientation.w


            q_aruco_rot = R.from_quat([qx, qy, qz, qw]) # builds a rot object from given array
            q_offset_rot = R.from_quat([-0.7071, 0.7071, 0, 0])

            q_world_rot = q_offset_rot * q_aruco_rot   # scipy has * as rotation composition
            result = q_world_rot.as_quat()             # returns [x, y, z, w] array

            #self.qx, self.qy, self.qz, self.qw = result
            self.qx=result[0]
            self.qy=result[1]
            self.qz=result[2]
            self.qw=result[3]
            # coordinates
            q_offset = R.from_quat([-0.7071, 0.7071, 0, 0])   # same offset as before
            pos_vec = np.array([temp_x, temp_y, temp_z])
            world_pos = q_offset.apply(pos_vec)
            self.x_current, self.y_current, self.z_current= world_pos

            #now lets publish this to the sequential node
            coord=Pose()

            # Assign position coordinates
            coord.position.x = self.x_current
            coord.position.y = self.y_current
            coord.position.z = self.z_current

        # Assign orientation (neutral/identity rotation)
            coord.orientation.x = self.qx
            coord.orientation.y = self.qy
            coord.orientation.z = self.qz
            coord.orientation.w = self.qw

            self.coord_publisher_.publish(coord)
            #log time (for us a dt is every time we get a new pose from the camera)
            
            
            






    

    def target_callback(self, FMAmsg):
        #FMAmsg = Float64MultiArray() # with this we can specify the target position
        # also PoseStamped can be used for this purpose
        #FMAmsg.data = [5.0, 5.0, 0.0]
        #self.subscriber_odom_.publish(FMAmsg)
         #all this is not needed if we are getting this FMAmsg from a terminal or another node
        self.target_x = FMAmsg.data[0]
        self.target_y = FMAmsg.data[1]





def main(args=None):  

    rclpy.init(args=args)
    node = diff_PID()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    