#include "iostream"
#include "memory"

//using namespace std;

int main()
{
    auto p1 = std::make_shared<std::string>("This is a str.");   //std：：make_shared<数据类型/类>（参数）
    std::cout<<"p1的饮用计数:"<<p1.use_count()<<",指向的内存地址"<<p1.get()<<std::endl;

    auto p2 = p1;
    std::cout<<"p1的饮用计数:"<<p1.use_count()<<",指向的内存地址"<<p1.get()<<std::endl;
    std::cout<<"p2的饮用计数:"<<p2.use_count()<<",指向的内存地址"<<p2.get()<<std::endl;

    p1.reset();//释放饮用，不指向'This is a str.'所在内存
    std::cout<<"p1的饮用计数:"<<p1.use_count()<<",指向的内存地址"<<p1.get()<<std::endl;
    std::cout<<"p2的饮用计数:"<<p2.use_count()<<",指向的内存地址"<<p2.get()<<std::endl;

    std::cout<<"p2指向的内存地址数据"<<p2->c_str()<<std::endl;//调用成员方法

    return 0;
}