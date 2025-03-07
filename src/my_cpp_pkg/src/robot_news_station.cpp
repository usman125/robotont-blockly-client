#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;

class RobotNewsStationNode : public rclcpp::Node{
public:
    RobotNewsStationNode() : Node("robot_news_station"), robot_name_("R2D2"){
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
        _timer = this->create_wall_timer(0.5s, std::bind(&RobotNewsStationNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Robot news station has been started.");
    }
private:
    void publishNews(){
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hi, I am ") + robot_name_ + std::string(" from robot news station");
        publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    std::string robot_name_;
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}