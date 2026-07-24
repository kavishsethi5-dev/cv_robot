#!/usr/bin/env python3
from geometry_msgs import msg
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from scipy.spatial.transform import Rotation as R
import numpy as np

class box_target(Node):
    def __init__(self):
        super().__init__('box_target')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/box_target', 10)
        self.create_subscription(ArucoMarkers, '/aruco_markers_box', self.callback, 10)
        
    def callback(self,msg):
        if 0 not in msg.marker_ids:
            return
        t_x=msg.poses[0].position.x
        t_y=msg.poses[0].position.y
        t_z=msg.poses[0].position.z
        o_x=msg.poses[0].orientation.x
        o_y=msg.poses[0].orientation.y
        o_z=msg.poses[0].orientation.z
        o_w=msg.poses[0].orientation.w
        q_aruco_rot = R.from_quat([o_x, o_y, o_z, o_w]) # builds a rot object from given array
        q_offset_rot = R.from_quat([-0.7071, 0.7071, 0, 0])

        q_world_rot = q_offset_rot * q_aruco_rot   # scipy has * as rotation composition
        result = q_world_rot.as_quat()             # returns [x, y, z, w] array

        #self.qx, self.qy, self.qz, self.qw = result
        qx=result[0]
        qy=result[1]
        qz=result[2]
        qw=result[3]
        # coordinates
        q_offset = R.from_quat([-0.7071, 0.7071, 0, 0])   # same offset as before
        pos_vec = np.array([t_x, t_y, t_z])
        world_pos = q_offset.apply(pos_vec)+ np.array([0.0, 0.0, 10.0])
        # after he rotation, the camera is facing -z so we just add 20m 
        x, y, z= world_pos

        #now to publish this
        fma_msg=Float64MultiArray()
        #self.current_x = fma_msg.data[0], way to extract
        fma_msg.data=[x,y,z,qx,qy,qz,qw]
        #publish
        self.publisher_.publish(fma_msg)

def main(args=None):  

    rclpy.init(args=args)
    node = box_target()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            



