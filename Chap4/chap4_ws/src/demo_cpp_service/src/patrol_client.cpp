//客户端节点，生成随机点，向服务端请求
#include "rclcpp/rclcpp.hpp"
#include "chap4_interfaces/srv/patrol.hpp"
#include <chrono>
#include <ctime>
//参数化头文件
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

using SetP = rcl_interfaces::srv::SetParameters;
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

    /** 创建客户端发送请求，返回结果*/
    SetP::Response::SharedPtr call_set_parameters(const rcl_interfaces::msg::Parameter &param)
    {
        auto param_client = this -> create_client<SetP>("/turtle_controller/set_parameters");
            //检测服务是否上线
            while (!param_client -> wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(this ->get_logger(),"等待服务上线过程中，rclcpp已停止运行，客户端已停止");
                    return nullptr;
                }
                RCLCPP_INFO(this ->get_logger(),"等待服务上线中");
            }
            //1.构造请求的对象
            auto request = std::make_shared<SetP::Request>();
            request->parameters.push_back(param);
            //2.发送请求
            auto future = param_client -> async_send_request(request);
            rclcpp::spin_until_future_complete(this -> get_node_base_interface(),future);
            auto response = future.get();
            return response;
    }

    /*更新参数k*/
    void update_server_param_k(double k)
    {
        //1.创建参数对象
        auto param = rcl_interfaces::msg::Parameter();
        param.name = "k";
        //2.创建参数值
        auto param_value = rcl_interfaces::msg::ParameterValue();
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_value.double_value = k;
        param.value = param_value;
        //3.使用服务发送请求，并处理结果
        RCLCPP_INFO(this->get_logger(),"正在请求更新参数%s为%f",param.name.c_str(),k);
        auto response = this->call_set_parameters({param});
        if(response == nullptr)
        {
            RCLCPP_WARN(this->get_logger(),"参数更新请求失败，未收到服务端响应");
            return;
        }
        for (const auto &result : response->results) 
        //基于范围的for循环,以只读的方式（const）、（&别名）遍历 response->results 容器中的每一个元素，并在每次循环中将当前元素命名为result供使用
        {
            if (!result.successful)
            {
                RCLCPP_ERROR(this->get_logger(), "参数更新失败, 原因: %s", result.reason.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "参数更新成功");
            }
        }
    }


private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PatrolClient>();
    node->update_server_param_k(4.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}