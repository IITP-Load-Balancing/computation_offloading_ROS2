#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <sstream>

class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher(const std::string & node_name, const std::string & topic_name)
    : Node(node_name), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SimplePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        std_msgs::msg::String message;
        message.data = "Data#" + std::to_string(count_++);

        publisher_->publish(message);

        if (count_ > 10) {
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc != 3) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Usage: ros2 run simple_pub <node_name> <topic_name>");
        return 1;
    }

    std::string node_name = argv[1];
    std::string topic_name = argv[2];

    rclcpp::spin(std::make_shared<SimplePublisher>(node_name, topic_name));

    rclcpp::shutdown();
    return 0;
}
