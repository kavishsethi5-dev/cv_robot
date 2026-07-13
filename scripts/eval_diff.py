#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import time 
import math
import numpy as np
import matplotlib.pyplot as plt

class EvalDiff(Node):
    def __init__(self):
        super().__init__('eval_diff')
        self.position_sub_=self.create_subscription(Pose, 'camera_transformed_coord', self.position_callback, 10)
        self.target_sub_=self.create_subscription(Float64MultiArray, '/target',self.target_callback, 10)
        self.create_timer(0.01, self.check_callback) #check arrival and store metrics
        #Initialize variables
        self.current_x=None
        self.current_y=None
        self.start_x=None
        self.start_y=None
        self.target_x=None
        self.target_y=None
        self.start_time=None #not zero
        self.elapsed = 0.0
        self.live_time=None #not zero
        self.live_time_=None
        self.Accumulated_path_length=0
        self.last_x = None
        self.last_y = None
        self.last_yaw_angle=None
        self.yaw_deg_current=None
        self.accumulated_angle = 0.0
        self.accumulated_angle_copy = 0.0
        self.idx = 0
        self.distance_idx=None
        
        self.d=[]
        self.t=[]
        self.a=[]
        self.x=[]
        self.y=[]
        self.e=[]



    def position_callback(self,msg):
        #best place to add live time
        if self.live_time is None:
            self.live_time=self.get_clock().now()
            return
        self.current_x= msg.position.x
        self.current_y= msg.position.y
        self.live_time_= (self.get_clock().now() - self.live_time).nanoseconds / 1e9

        if self.last_x is None:
            self.last_x= self.current_x
            self.last_y= self.current_y
            return

        distance= math.hypot(self.current_x - self.last_x, self.current_y - self.last_y)

        #setting current value as last value
        self.last_x= self.current_x
        self.last_y= self.current_y

        self.Accumulated_path_length += distance

        #
        q = msg.orientation

        self.x_val = q.x
        self.y_val = q.y
        self.z_val = q.z
        self.w_val = q.w

        self.yaw_rad = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.yaw_deg_current= math.degrees(self.yaw_rad)

        if self.last_yaw_angle is None:
            self.last_yaw_angle=self.yaw_deg_current
            return
        
        angle_delta= self.yaw_deg_current-self.last_yaw_angle
        self.last_yaw_angle=self.yaw_deg_current #after difference make current angle last angle

        #normalize
        angle_delta = (angle_delta + 180) % 360 - 180

        self.accumulated_angle += angle_delta
        self.accumulated_angle_copy += angle_delta #same angle just wont set this to zero like first 
    
        




    
    def target_callback(self,msg): 
    # this will run only when robot reaches prev target as given in seq_diff target
        
        if self.target_x is None:
            start_x=self.current_x
            start_y=self.current_y
        elif self.target_x is not None:
            start_x=self.target_x
            start_y=self.target_y
    #now running after this logic, changing target to new target
        self.target_x = msg.data[0]
        self.target_y = msg.data[1]
        #time, reach a target
        self.start_time = self.get_clock().now() #clock not time
        self.idx +=1
        #so old target is saved as start_x and new target as new target(self.target_x)
        self.distance_idx=math.hypot(self.current_x - start_x , self.current_y - start_y)
        #we can report the self.accumulated_angle and then set it to zero
        self.angle_idx=self.accumulated_angle
        #setting acc angle to zero after target reached
        self.accumulated_angle=0.0
        self.get_logger().info(f' Target no. {self.idx}: distance traveled: {self.distance_idx:.2f}m')
        #now as soon as target changes it gives us accumulated path length from current position to old target
        self.get_logger().info(f' Target no. {self.idx}: angle: {self.angle_idx:.2f}deg')
        #angle delta from current position to old target
        


    def report(self):
        if self.start_time is None: 
            #nothing
            #This is to stop error because of check_callback subtracting none
            return
        if math.hypot(self.current_x - self.target_x , self.current_y - self.target_y) < 0.1:
            Reached = True
            if Reached == True: #here if Reached: will suffice instead also
                #we use if Reached is not False:
                self.elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                self.get_logger().info(f'Target {self.idx} reached in {self.elapsed:.2f}s')
                # return removed so below code works in both cases
        #here we can store and report data for the graphs (we could  do in position one also but
        # im doing here since it already has the logic for time)
        # e-t graph
        error = math.hypot(self.current_x - self.target_x, self.current_y - self.target_y)
        self.e.append(error)
        self.a.append(self.accumulated_angle_copy)
        self.t.append(self.live_time_) #not self.elapsed bec that is reported only when target reached
        self.x.append(self.current_x)
        self.y.append(self.current_y)
       
        


            
    def check_callback(self): #for calling report
        if self.current_x is None or self.target_x is None:
            #nothing 
            return
        self.report()
    
    
    #plotting
def plot_metrics(t, e, a, x, y):
 
    t = np.array(t)
    #d = np.array(d) 
    a = np.array(a)
    x = np.array(x)
    y = np.array(y)
    e = np.array(e)
 
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Differential Drive Robot — Navigation Metrics', fontsize=14, fontweight='bold')
 
    # Top left — Distance vs Time
    axes[0, 0].plot(t, e, color='steelblue', linewidth=1.5)
    axes[0, 0].set_title('Error vs Time')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Error (m)')
    axes[0, 0].grid(True, alpha=0.3)
 
    # Top right — Angle vs Time
    axes[0, 1].plot(t, a, color='darkorange', linewidth=1.5)
    axes[0, 1].set_title('Accumulated Rotation vs Time')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Angle (deg)')
    axes[0, 1].grid(True, alpha=0.3)
 
    # Bottom left — X and Y vs Time on same graph
    axes[1, 0].plot(t, x, color='crimson', linewidth=1.5, label='X')
    axes[1, 0].plot(t, y, color='seagreen', linewidth=1.5, label='Y')
    axes[1, 0].set_title('X and Y Position vs Time')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Position (m)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
 
    # Bottom right — X vs Y (trajectory)
    axes[1, 1].plot(x, y, color='mediumpurple', linewidth=1.5)
    axes[1, 1].plot(x[0], y[0], 'go', markersize=8, label='Start')   # green dot at start
    axes[1, 1].plot(x[-1], y[-1], 'rs', markersize=8, label='End')   # red square at end
    axes[1, 1].set_title('Trajectory (X vs Y)')
    axes[1, 1].set_xlabel('X (m)')
    axes[1, 1].set_ylabel('Y (m)')
    axes[1, 1].legend()
    axes[1, 1].set_aspect('equal')   # so the path isn't distorted
    axes[1, 1].grid(True, alpha=0.3)
 
    plt.tight_layout()
    plt.savefig('robot_metrics.png', dpi=150, bbox_inches='tight')
    plt.show()


            
    
            
def main(args=None):  

    rclpy.init(args=args)
    node = EvalDiff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plot_metrics(node.t, node.e, node.a, node.x, node.y)
        rclpy.shutdown()
    
    
    

if __name__ == '__main__':
    main()

        


        
        

