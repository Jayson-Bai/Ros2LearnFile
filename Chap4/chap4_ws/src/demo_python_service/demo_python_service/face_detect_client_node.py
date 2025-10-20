import rclpy
from rclpy.node import Node
from chap4_interfaces.srv import FaceDetector

import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os

from cv_bridge import CvBridge
import time

#参数化 导入消息接口
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType

class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.bridge = CvBridge()
        self.Faces_png_path = os.path.join(get_package_share_directory('demo_python_service'),'resource/Faces.png')
        self.get_logger().info("人脸检测客户端已经启动")
        self.client = self.create_client(FaceDetector, 'face_detect')
        self.image = cv2.imread(self.Faces_png_path)

    #调用服务修改参数值
    def call_set_parameters(self,parameters):
        #1.创建一个客户端，等待服务上线
        update_param_client = self.create_client(SetParameters,'/face_detect_node/set_parameters')
        while update_param_client.wait_for_service(timeout_sec = 1.0) is False:
            self.get_logger().info("等待参数更新服务端上线")
        #2.创建request
        request = SetParameters.Request()
        request.parameters = parameters
        #3.调用服务端更新参数
        future = update_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response
    
    def update_detect_model(self,model='hog'):
        #根据传入的model，构造parameters，然后调用call_set_parameters更新服务端的参数
        #1.创建参数对象
        param = Parameter()
        param.name = 'model'
        #2.赋值
        param_value = ParameterValue()
        param_value.string_value = model
        param_value.type = ParameterType.PARAMETER_STRING
        param.value = param_value
        #3.请求更新参数
        response = self.call_set_parameters([param])
        for result in response.results:
            if result.successful:
                self.get_logger().info(f"设置参数结果：{result.successful}{result.reason}")

        

    def send_request(self):
        while self.client.wait_for_service(timeout_sec = 1.0) is False:
            self.get_logger().info("等待服务端上线")
        
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
    node.update_detect_model('hog')
    node.send_request()
    node.update_detect_model('cnn')
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()
