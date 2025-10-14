//服务节点，订阅了 海龟当前的位置+姿态，发布 控制小海龟朝设定的目标点移动
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "chap4_interfaces/srv/patrol.hpp"

using Patrol = chap4_interfaces::srv::Patrol;

class TurtleController : public rclcpp::Node //TurtleController类 定义
{
//声明私有变量
private:
        rclcpp::Service<Patrol>::SharedPtr patrol_service_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_; 
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_; 
        double target_x_{1.0}; //初始值
        double target_y_{1.0};
        double k_{1.0};
        double max_speed_{3.0};

//创建TurtleController对象，
public:
    TurtleController() : Node("turtle_controller") //构造函数，初始化其作为ros2节点的属性，并命名为turtle_controller
    {
        patrol_service_ = this->create_service<Patrol> //调用从rclcpp::Node继承来的create_service方法创建服务，服务接口模板Patrol：
        ("patrol",  //服务名称
        [&](const Patrol::Request::SharedPtr request, Patrol::Response::SharedPtr response) -> void //Lambda函数，服务的回调函数    
        //[]：Lambda捕获列表，&：以引用方式捕获所有外部变量   
            {
                if( (0 < request->target_x && request->target_x < 12.0f) && (0 < request->target_y && request->target_y< 12.0f) )
                {
                    this ->target_x_ = request->target_x;
                    this ->target_y_ = request->target_y;
                    response->result = Patrol::Response::SUCCESS; //响应发送到客户端
                }
                else
                {
                    response->result = Patrol::Response::FAIL;
                } 
            }
        );
        //创建发布者，发布消息接口geometry_msgs::msg::Twist，topic名称/turtle1/cmd_vel，服务队列大小10
        velocity_publisher_ = this -> create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10); 
        //创建订阅者，订阅消息类型turtlesim::msg::Pose，订阅小乌龟模拟器运行后产生的位置话题/turtle1/pose，队列大小10，回调函数
        pose_subscription_ = this -> create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&TurtleController::on_pose_received_,this,std::placeholders::_1));
        
    }
private:
    //订阅者的回调函数，控制核心
    void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose)
    {
        auto message = geometry_msgs::msg::Twist(); //调用Twist类的默认构造函数，创建Twist消息对象message，所有值归零

        //1.记录当前位置
        double current_x = pose -> x;
        double current_y = pose -> y;
        RCLCPP_INFO(this->get_logger(),"当前位置:x=%f,y=%f",current_x,current_y);

        //2.计算当前位置与目标位置的距离差与角度差
        double distance = std::sqrt(
            (target_x_-current_x)*(target_x_-current_x)+
            (target_y_-current_y)*(target_y_-current_y)
        );

        double angle = std::atan2((target_y_-current_y),(target_x_-current_x)) - pose->theta; 

        //3.控制策略
        if(distance > 0.1){
            if(fabs(angle)>0.2){
                message.angular.z = fabs(angle);
            }else{
                message.linear.x = k_*distance;
            }
        }

        if(message.linear.x > max_speed_){
            message.linear.x = max_speed_;
        }
        
        //调用publish方法发布速度角度信息
        velocity_publisher_ -> publish(message);    
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}