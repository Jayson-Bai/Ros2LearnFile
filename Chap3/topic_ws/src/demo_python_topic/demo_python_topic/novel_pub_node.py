import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String   #消息接口
from queue import Queue

class NovelPubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name},start!')
        self.novel_queue_ = Queue()  #创建队列 #注意要放在timber前面
        self.novel_publisher_ = self.create_publisher(String,'novel',10)    #创建发布
        self.create_timer(5,self.timber_callbak)  #创建定时器
        

    def timber_callbak(self):
        #self.novel_publisher_.publish()
        if self.novel_queue_.qsize()>0:
            line = self.novel_queue_.get()
            msg = String() #组装
            msg.data = line
            self.novel_publisher_.publish(msg)
            self.get_logger().info(f'Published:{msg}')

    def download(self, url):
        response = requests.get(url)
        response.encoding = 'utf-8'
        text = response.text
        self.get_logger().info(f'Downloading{url},{len(text)}')
        for line in text.splitlines():
            self.novel_queue_.put(line)
       
def main():
    rclpy.init()
    node = NovelPubNode('novel_pub')
    node.download('http://0.0.0.0:8000/novel1.txt') 
    rclpy.spin(node)
    rclpy.shutdown()
