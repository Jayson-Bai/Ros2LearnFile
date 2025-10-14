import rclpy
from rclpy.node import Node
from chap4_interfaces.srv import FaceDetector

import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os

from cv_bridge import CvBridge
import time

class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.bridge = CvBridge()
        self.Faces_png_path = os.path.join(get_package_share_directory('demo_python_service'),'resource/Faces.png')
        self.get_logger().info("人脸检测客户端已经启动")
        self.client = self.create_client(FaceDetector, 'face_detect')
        self.image = cv2.imread(self.Faces_png_path)

    def send_request(self):
        while self.client.wait_for_service(timeout_sec = 1.0) is False:
            self.get_logger().info("等待服务上线")
        
        #构造request
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)

        #发送请求并等待处理完成
        future = self.client.call_async(request) #现在的future中并不包含响应，需要等待服务端处理完成才会把结果放入

        #方法一：
        #rclpy.spin_until_future_complete(self, future) #等待服务端返回响应
        #response = result_future.result()
        #response = future.result() #获得响应
        #self.get_logger().info(f"接收到响应，共检测到{response.number}张人脸，耗时{response.use_time}s")
        #self.show_response(response)

        #方法二：
        def result_callback(result_future):
            response = result_future.result()
            self.get_logger().info(f"接收到响应，共检测到{response.number}张人脸，耗时{response.use_time}s")
            self.show_response(response)
            
        future.add_done_callback(result_callback)


    def show_response(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image,(left,top),(right,bottom),(255,0,0),4)

        cv2.imshow('Face detect result', self.image)
        cv2.waitKey(0)

def main():
    rclpy.init()
    node = FaceDetectClientNode()
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()
