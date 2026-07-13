#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose

class seq_diff_target(Node):
    def __init__(self):
        super().__init__('seq_diff_target')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/target', 10)
        self.create_subscription(Pose, 'camera_transformed_coord', self.callback, 10)
        self.targets = [
            (5.0, 4.0),
            (5.0, -5.0),
            (-5.0, -5.0),
            (-5.0, 5.0),
            (0.0, 0.0),
            (-7.0, 7.0),
        ]
        self.idx = 0
        self.timer = self.create_timer(0.05, self.check_arrival)
        self.current_target_x = None
        self.current_target_y = None
        self.x_current = 0.0
        self.y_current = 0.0
        self.create_timer(2.0, self.send_first)  # wait for publisher to be ready
        self.first_sent = False

    def send_first(self):
        if not self.first_sent: # not False = True, so we enter the block
            self.send_next() #sent
            self.first_sent = True #now its true

    def send_next(self):
        if self.idx >= len(self.targets):
            self.get_logger().info("All targets sent.")
            return
        x, y = self.targets[self.idx]
        msg = Float64MultiArray()
        msg.data = [x, y]
        self.publisher_.publish(msg)
        self.current_target_x = x
        self.current_target_y = y
        self.get_logger().info(f"Sent target {self.idx}: ({x}, {y})")
        self.idx += 1

    def check_arrival(self):
        if self.current_target_x is None:
            return
        import math
        dist = math.sqrt((self.current_target_x - self.x_current)**2 +
                         (self.current_target_y - self.y_current)**2)
        if dist < 0.30:
            self.send_next()
        if dist < 0.5:
            self.get_logger().info(f'dist to target: {dist:.3f}')

    def callback(self, msg):
        self.x_current = msg.position.x
        self.y_current = msg.position.y
       

def main(args=None):  

    rclpy.init(args=args)
    node = seq_diff_target()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    