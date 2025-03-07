#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node{
public:
    MyNode() : Node("cpp_test"), _counter(0){
        RCLCPP_INFO(this->get_logger(), "hello world from cpp");
        _timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyNode::timerCallBack, this));
    }
private:
    void timerCallBack(){
        RCLCPP_INFO(this->get_logger(), "Hello timer %d", _counter);
        _counter++;
    }
    rclcpp::TimerBase::SharedPtr _timer;
    int _counter;
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}