#include <iostream>
#include <algorithm>

int main()
{
    auto add = [](int a, int b) -> int {return a+b;};//lambda表达式语法
    int sum = add(200,50);
    auto print_sum = [sum]() -> void
    {
        std::cout << sum << std::endl;
    };

    print_sum();
    return 0;
}