//cpp发布topic控制小海龟到达指定位置
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TurtleControlNode: public rclcpp:: Node
{
private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; //发布者的智能指针
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_; //订阅者的智能共享指针
        double target_x_{1.0};
        double target_y_{1.0};
        double k_{1.0};
        double max_speed_{3.0};
        
    
public:
    explicit TurtleControlNode(const std::string& node_name):Node(node_name)   //隐式转换：……
    {
        publisher_ = this -> create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
        subscriber_ = this -> create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&TurtleControlNode::on_pose_received_,this,std::placeholders::_1));
        
    }

    //订阅者的回调函数：参数————收到数据的共享指针
    void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose)
    {
        //获取当前位置
        auto current_x = pose -> x;
        auto current_y = pose -> y;
        RCLCPP_INFO(get_logger(),"当前:x=%f,y=%f",current_x,current_y);

        //计算当前位置与目标位置的距离差与角度差
        auto distance = std::sqrt(
            (target_x_-current_x)*(target_x_-current_x)+
            (target_y_-current_y)*(target_y_-current_y)
        );

        auto angle = std::atan2((target_y_-current_y),(target_x_-current_x)) - pose->theta;

        //控制策略
        auto msg = geometry_msgs::msg::Twist();
        if(distance > 0.1){
            if(fabs(angle)>0.2){
                msg.angular.z = fabs(angle);
            }else{
                msg.linear.x = k_*distance;
            }
        }

        if(msg.linear.x > max_speed_){
            msg.linear.x = max_speed_;
        }

        publisher_ -> publish(msg);    
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleControlNode>("turtle_control");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}