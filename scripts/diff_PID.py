#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import math
import numpy as np
from ros2_aruco_interfaces.msg import ArucoMarkers
from scipy.spatial.transform import Rotation as R

# For safety though dictionary are diff so it shouldnt interfere (5x5 here)
MARKER_ID = 1

# 2 is too slow causes it to stop for now 3.25
TIMEOUT = 2.75

# for  now 0.50 so that the arm has space.
ARRIVAL_THRESHOLD = 0.50


class diff_PID(Node):

    def __init__(self):
        super().__init__('diff_PID')

        self.create_subscription(ArucoMarkers, '/aruco_markers', self.odom_callback, 10)
        self.subscriber_target_ = self.create_subscription(
            Float64MultiArray, '/box_target', self.target_callback, 10)
        self.publishers_cmd_vel_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.coord_publisher_ = self.create_publisher(Pose, 'camera_transformed_coord', 10)

        self.kp = 1.2 #gains sch
        self.ki = 0.01
        self.kd = 0.05
        self.kp_a = 0.8
        self.ki_a = 0.01
        self.kd_a = 0.05

        self.max_linear = 1.0
        self.max_angular = 0.45


        
        self.max_error_sum = 2.0
        self.max_error_sum_a = 2.0

        
        self.last_error = 0.0
        self.error_sum = 0.0
        self.last_error_a = 0.0
        self.error_sum_a = 0.0

        self.x_current = 0.0
        self.y_current = 0.0
        self.z_current = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        self.target_x = None
        self.target_y = None

        self.last_time = None
        self.odom_last_time = None
    

        # Stopper only. The control loop itself is driven by odom_callback,
        # so it runs exactly once per new position measurement.
        self.stopper = self.create_timer(0.2, self.stopper_callback)

    ###no longer separate timer, called from odom_callback
    def run_control(self):
        if self.target_x is None:
            return

        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            return

        time_step = (now - self.last_time).nanoseconds / 1e9
        if time_step <= 0.001:
            return
        
        time_step = min(time_step, 0.5)
        self.last_time = now

        # Heading from quaternion (yaw only)
        siny_cosp = 2 * (self.qw * self.qz + self.qx * self.qy)
        cosy_cosp = 1 - 2 * (self.qy * self.qy + self.qz * self.qz)
        theta_current = math.atan2(siny_cosp, cosy_cosp)

        dx = self.target_x - self.x_current
        dy = self.target_y - self.y_current

        distance_error = math.hypot(dx, dy)
        alpha = math.atan2(dy, dx)

        angle_error = alpha - theta_current
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Arrival check 
        if distance_error < ARRIVAL_THRESHOLD:
            self.publishers_cmd_vel_.publish(Twist())
            # Reset all so thats its not a jagged path
            self.error_sum = 0.0
            self.error_sum_a = 0.0
            self.last_error = 0.0
            self.last_error_a = 0.0
            self.get_logger().info(f"ARRIVED: dist={distance_error:.3f}")
            return

        # linear
        self.error_sum += distance_error * time_step
        self.error_sum = max(-self.max_error_sum, min(self.max_error_sum, self.error_sum))
        P = self.kp * distance_error
        I = self.ki * self.error_sum
        D = self.kd * (distance_error - self.last_error) / time_step
        self.last_error = distance_error

        # angular
        self.error_sum_a += angle_error * time_step
        self.error_sum_a = max(-self.max_error_sum_a, min(self.max_error_sum_a, self.error_sum_a))
        P_a = self.kp_a * angle_error
        I_a = self.ki_a * self.error_sum_a
        D_a = self.kd_a * (angle_error - self.last_error_a) / time_step
        self.last_error_a = angle_error

        pid_dist = max(0.0, min(self.max_linear, P + I + D))
        pid_angle = max(-self.max_angular, min(self.max_angular, P_a + I_a + D_a))

        
        # if slow
        if abs(angle_error) < math.radians(5.0):
            pid_angle = 0.0

        # coupled
        heading_factor = max(0.0, math.cos(angle_error))

        fmsg = Twist()
        fmsg.linear.x = pid_dist * heading_factor
        fmsg.angular.z = pid_angle
        self.publishers_cmd_vel_.publish(fmsg)

        self.get_logger().info(
            f"pos=({self.x_current:.2f}, {self.y_current:.2f}) "
            f"dist={distance_error:.2f} "
            f"theta={math.degrees(theta_current):.1f} "
            f"alpha={math.degrees(alpha):.1f} "
            f"ang_err={math.degrees(angle_error):.1f} "
            f"dt={time_step:.3f}"
        )


    def odom_callback(self, msg):
        #for safety only
        if MARKER_ID not in msg.marker_ids:
            return

        idx = list(msg.marker_ids).index(MARKER_ID)
        pose = msg.poses[idx]

        q_aruco_rot = R.from_quat([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])
        q_offset = R.from_quat([-0.7071, 0.7071, 0, 0])

        result = (q_offset * q_aruco_rot).as_quat()
        self.qx, self.qy, self.qz, self.qw = result

        pos_vec = np.array([pose.position.x, pose.position.y, pose.position.z])
        self.x_current, self.y_current, self.z_current = q_offset.apply(pos_vec)

        coord = Pose()
        coord.position.x = self.x_current
        coord.position.y = self.y_current
        coord.position.z = self.z_current
        coord.orientation.x = self.qx
        coord.orientation.y = self.qy
        coord.orientation.z = self.qz
        coord.orientation.w = self.qw
        self.coord_publisher_.publish(coord)

        self.odom_last_time = self.get_clock().now()

        # RUN PID calcs and controller every detection not a separate timer
        self.run_control()

    def target_callback(self, FMAmsg):
        self.target_x = FMAmsg.data[0]
        self.target_y = FMAmsg.data[1]

    def stopper_callback(self):
         #Stop the robot if position updates dry up
        if self.odom_last_time is None:
            return
        time_out = (self.get_clock().now() - self.odom_last_time).nanoseconds / 1e9
        if time_out > TIMEOUT:
            self.publishers_cmd_vel_.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = diff_PID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publishers_cmd_vel_.publish(Twist())
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()