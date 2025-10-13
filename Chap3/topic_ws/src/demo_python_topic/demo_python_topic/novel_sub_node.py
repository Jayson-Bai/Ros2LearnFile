import espeakng #朗读
import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String   #消息接口
from queue import Queue
import threading   
import time

class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name},start')
        self.novel_queue_ = Queue()
        self.novel_subscriber_ = self.create_subscription(String,'novel',self.novel_callback,10)  #‘novel’话题名称必须与发布时一模一样   创建订阅
        self.speech_thread_ = threading.Thread(target=self.speak_thread)
        self.speech_thread_.start() #注意线程启动

    def novel_callback(self,msg):
        self.novel_queue_.put(msg.data)

    def speak_thread(self):
        speaker = espeakng.Speaker()
        speaker.voice = 'zh'

        while rclpy.ok():
            if self.novel_queue_.qsize()>0:
                text = self.novel_queue_.get()
                self.get_logger().info(f'Reading:{text}')
                speaker.say(text)
                speaker.wait()
            else:
                time.sleep(1)
                
        

def main():
    rclpy.init()
    node = NovelSubNode('novel_pub')
    rclpy.spin(node)
    rclpy.shutdown()
