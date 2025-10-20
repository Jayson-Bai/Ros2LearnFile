import rclpy
from rclpy.node import Node
from chap4_interfaces.srv import FaceDetector

import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os

from cv_bridge import CvBridge
import time

class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.Faces_png_path = os.path.join(get_package_share_directory('demo_python_service'),'resource/Faces.png')
        self.bridge = CvBridge()
        #参数固定写死
        #self.number_of_times_to_upsample = 1
        #self.model ='hog'

        #参数化
        self.declare_parameter('number_of_times_to_upsample',1)
        self.declare_parameter('model','hog')
        self.number_of_times_to_upsample = self.get_parameter('number_of_times_to_upsample').value
        self.model = self.get_parameter('model').value

        self.service_ = self.create_service(FaceDetector, 'face_detect', self.detect_face_callback)



    def detect_face_callback(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.Faces_png_path)
            self.get_logger().info(f"传入图像为空，使用默认图像")
        
        start_time = time.time()
        self.get_logger().info(f"加载完成图像，开始识别")
        face_locations = face_recognition.face_locations(cv_image, number_of_times_to_upsample =self.number_of_times_to_upsample, model=self.model)
        response.use_time = time.time() - start_time
        response.number = len(face_locations)

        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)

        return response


def main():
    rclpy.init()
    node = FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()



