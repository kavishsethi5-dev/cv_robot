#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class diff_target(Node):

    def __init__(self):
        super().__init__('diff_target')
        
        self.diff_target_pub_=self.create_publisher(Float64MultiArray, '/target', 10)

        self.timer = self.create_timer(1.0, self.on_timer_check)


    def on_timer_check(self):

        self.get_logger().info("target sent")
        
        msg = Float64MultiArray() 
    
        msg.data = [10.0, -15.0, 0.0]
        self.diff_target_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = diff_target()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()