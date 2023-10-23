#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <sstream>

class StaticPayloadPublisher : public rclcpp::Node
{
public:
    StaticPayloadPublisher(const std::string & node_name, const std::string & topic_name, std::chrono::milliseconds period, std::size_t payload_size)
    : Node(node_name), count_(0), payload_size_(payload_size)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
        timer_ = this->create_wall_timer(period, std::bind(&StaticPayloadPublisher::timer_callback, this));
    }

void timer_callback()
{
    std_msgs::msg::String message;
    std::string prefix = "Data#" + std::to_string(count_) + " ";
    std::size_t prefix_size = prefix.size();
    if (payload_size_ >= prefix_size) {
        message.data = prefix + std::string(payload_size_ - prefix_size, 'a');
    } else {
        message.data = prefix;
    }
    auto now = this->now();
    std::string output = std::to_string(count_++) + " " + std::to_string(now.nanoseconds());
    std::cout << output << std::endl;
    publisher_->publish(message);

    if (count_ >= 20) {
        rclcpp::shutdown();
    }
}

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::size_t count_;
    std::size_t payload_size_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 4) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Usage: ros2 run node_gen static_payload_pub <node_name> <topic_name> <period_millisec> <payload_size>");
        return 1;
    }

    std::string node_name = argv[1];
    std::string topic_name = argv[2];
    std::chrono::milliseconds period(std::stoi(argv[3]));
    std::size_t payload_size = std::stoi(argv[4])*1024;

    rclcpp::spin(std::make_shared<StaticPayloadPublisher>(node_name, topic_name, period, payload_size));
    return 0;
}
