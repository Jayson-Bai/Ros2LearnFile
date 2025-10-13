#面向对象编程
import rclpy
from rclpy.node import Node
class PersonNode(Node):
    def __init__(self,node_name: str,name_value: str, age_value: int) -> None:
        print('PersonNode __init__方法被调用了，添加了两个属性')
        super().__init__(node_name)
        self.name = name_value
        self.age = age_value

    def eat(self,food_name: str):
        """""
        方法：吃东西
        """""
        #print(f"{self.name},{self.age}岁，爱吃{food_name}")
        self.get_logger().info(f"{self.name},{self.age}岁，爱吃{food_name}")


def main():
    rclpy.init()
    node = PersonNode('carrot','白萝卜',23)
    node.eat('大鸡腿')
    node1 = PersonNode('black_carrot','黑萝卜',23)
    node1.eat('大鸡排')
    rclpy.spin(node)
    rclpy.shutdown()