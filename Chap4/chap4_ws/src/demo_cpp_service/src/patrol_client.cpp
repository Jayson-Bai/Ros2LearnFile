//客户端节点，生成随机点，向服务端请求
#include "rclcpp/rclcpp.hpp"
#include "chap4_interfaces/srv/patrol.hpp"
#include <chrono>
#include <ctime>

using Patrol = chap4_interfaces::srv::Patrol;
using namespace std::chrono_literals; //可以使用m ms

class PatrolClient : public rclcpp::Node
{
public:
    PatrolClient() : Node("turtle_controller") 
    {
        srand(time(NULL)); //初始化随机数种子
        patrol_client_ = this -> create_client<Patrol>("patrol");
        timer_ = this -> create_wall_timer(10s,[&]()->void{
            //检测服务是否上线
            while (!this -> patrol_client_ -> wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(this ->get_logger(),"等待服务上线过程中，rclcpp已停止运行，客户端已停止");
                    return;
                }
                RCLCPP_INFO(this ->get_logger(),"等待服务上线中");
            }
            //构造请求的对象
            auto request = std::make_shared<Patrol::Request>();
            request -> target_x = rand() % 15;
            request -> target_y = rand() % 15;
            RCLCPP_INFO(this ->get_logger(),"准备好目标点%f,%f",request -> target_x, request -> target_y);

            this -> patrol_client_ -> async_send_request(request, [&](rclcpp::Client<Patrol>::SharedFuture result_future) -> void{
                auto response = result_future.get();
                if(response -> result == Patrol::Response::SUCCESS)
                {
                    RCLCPP_INFO(this ->get_logger(),"请求巡逻目标点成功");
                }
                if(response -> result == Patrol::Response::FAIL)
                {
                    RCLCPP_INFO(this ->get_logger(),"请求巡逻目标点失败");
                }
            });


        });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PatrolClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}