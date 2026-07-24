import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import math
import time
import numpy as np
from sensor_msgs.msg import CameraInfo
from ros2_aruco_interfaces.msg import ArucoMarkers ###
from cv_robot.srv import ChangeRes
from ros_gz_interfaces.srv import DeleteEntity
from ros_gz_interfaces.srv import SpawnEntity
#ros_gz_interfaces/srv has std ros2 service types for gazebo, like spawn and delete entity

class res(Node):
    def __init__(self):
        super().__init__('res')
        self.flip_sub_ = self.create_subscription(Float64MultiArray, '/box_target',self.flip_callback, 10)
        self.srv = self.create_service(
            ChangeRes, 'change_res', self.callback_service)
        self.client = self.create_client(ChangeRes, 'change_res')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.yes=0
       
        #condition same so one cli only
        self.srv_delete = self.create_service(
            DeleteEntity, 'delete', self.delete_callback
        )
        self.srv_spawn = self.create_service(
            SpawnEntity, 'spawn', self.spawn_callback
        )
        self.cli_delete = self.create_client(DeleteEntity, 'delete')
        while not self.cli_delete.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.cli_spawn = self.create_client(SpawnEntity, 'spawn')
        while not self.cli_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')


    def send_request(self, width, height):
        req = ChangeRes.Request()
        req.r_msg.width = width
        req.r_msg.height = height
        #future = self.client.call_async(req) general
        #if box position has been detected then future is true
        if self.yes==1:
            future = self.client.call_async(req)
            
            #rclpy.spin_until_future_complete(self, future) we cant do this because it will block the node, so we use add_done_callback instead
            # this is because response is called from flip which is already spinning
            future.add_done_callback(self.response_callback)
            return future.result()

    def response_callback(self, future):
        response = future.result()
        self.get_logger().info(f'Success: {response.success}, Msg: {response.message}')

    def callback_service(self, request, response):
        response.success = True
        return response
    
    def flip_callback(self,target_msg):
    #the msg is coming from box_target
        if target_msg is not None:
            self.yes=1
            self.send_request(width,height)
            self.send_request_del(overhead_cam)
            self.send_request_spawn(name, sdf_filename, x, y, z, w)
            return
        
    
    def send_request_del(self,overhead_cam):
        req = DeleteEntity.Request()
        req.entity.name = 'overhead_cam.sdf'
        future = self.cli_delete.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    def send_request_spawn(self,name, sdf_filename, x, y, z, w):
        req = SpawnEntity.Request()
        req.entity_factory.name = 'overhead_camera_lowres'         
        req.entity_factory.sdf_filename ='overhead_camera_lowres.sdf'   
        req.entity_factory.pose.position.x = 0.0
        req.entity_factory.pose.position.y = 0.0
        req.entity_factory.pose.position.z = 10.0             # match whatever height you spawn it at today
        req.entity_factory.pose.orientation.w = 1.5708           # identity rotation; adjust if your camera needs pitch, same as -P 1.5708 in your launch args
        future = self.cli_spawn.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def delete_callback(self,request,response):
        response.success = True
        return response
    def spawn_callback(self,request,response):
        response.success = True
        return response


            
        


    




