from demo_python_pkg.person_node import PersonNode
import rclpy
from rclpy.node import Node

class WriterNode(PersonNode):
    def __init__(self,node_name:str, name:str, age:int, book_name: str) -> None:
        print('WriterNode __init__方法被调用了，添加了两个属性')
        super().__init__(node_name, name, age)
        self.book = book_name

    def represent_book(self):
        self.get_logger().info(f"代表书籍是{self.book}")

def main():
    rclpy.init()
    node = WriterNode('carrot','白萝卜',23,'如何减脂')
    node.eat('大鸡腿')
    node.represent_book()
    rclpy.spin(node)
    rclpy.shutdown()
    