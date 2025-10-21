import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener,Buffer #坐标监听器
from tf_transformations import euler_from_quaternion #四元数转欧拉角函数
import math #角度转弧度函数

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_,self)
        self.timer_ = self.create_timer(1.0,self.get_transform)

    def get_transform(self):
        """
        实时查询坐标关系
        """
        try:
            result = self.buffer_.lookup_transform('base_link','bottle_link',rclpy.time.Time(seconds=0),rclpy.time.Duration(seconds=1.0))
            transform = result.transform
            self.get_logger().info(f"平移：{transform.translation}")
            self.get_logger().info(f"旋转：{transform.rotation}")
            rotataion_euler = euler_from_quaternion([transform.rotation.x,transform.rotation.y,transform.rotation.z,transform.rotation.w])
            self.get_logger().info(f"旋转RPY：{rotataion_euler}")
        except Exception as e:
            self.get_logger().warn(f"获取坐标变换失败，原因：{str(e)}")

    
       

def main():
    rclpy.init()
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()
