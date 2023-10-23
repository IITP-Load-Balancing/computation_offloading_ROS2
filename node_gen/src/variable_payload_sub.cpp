#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <string>
#include <sstream>

class DynamicNode : public rclcpp::Node
{
public:
    DynamicNode(const std::string & node_name, const std::string & pub_topic_name, const std::string & sub_topic_name, long load, size_t depth)
    : Node(node_name), load_(load)
    {
        rclcpp::QoS qos(depth);
        publisher_ = this->create_publisher<std_msgs::msg::String>(pub_topic_name, qos);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            sub_topic_name, qos, std::bind(&DynamicNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void dummy_task(long load) {
        volatile double dummy_var = 1.0;  // volatile to prevent optimization
        for (long i = 0; i < load; i++) {
            dummy_var *= 1.00001;
        }
    }

    std::string extract_count(const std::string & data) {
        size_t start = data.find("Data#") + 5;
        size_t end = data.find(" ", start);
        return data.substr(start, end - start);
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string time = std::to_string(this->now().nanoseconds());  // convert to milliseconds

        dummy_task(load_);  // Add CPU load

        std::string count = extract_count(msg->data);

        std::string output = count + " " + time;
        if (count != "10") std::cout << output << std::endl;

        publisher_->publish(*msg);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    long load_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 5) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Usage: ros2 run node_gen variable_payload_sub <node_name> <sub_topic_name> <pub_topic_name> <dummy_task count> [qos_depth]");
        return 1;
    }

    std::string node_name = argv[1];
    std::string sub_topic_name = argv[2];
    std::string pub_topic_name = argv[3];
    long load = std::stol(argv[4]);

    size_t depth = 10;  // default QoS depth
    if (argc > 5) {
        depth = std::stoul(argv[5]);
    }

    rclcpp::spin(std::make_shared<DynamicNode>(node_name, pub_topic_name, sub_topic_name, load, depth));
    rclcpp::shutdown();
    return 0;
}
