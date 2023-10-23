#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>

class DynamicNode : public rclcpp::Node
{
public:
    DynamicNode(const std::string & node_name, const std::string & pub_topic_name, const std::string & sub_topic_name, long load)
    : Node(node_name), load_(load)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(pub_topic_name, 10);
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            sub_topic_name, 10, std::bind(&DynamicNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void dummy_task(long load) {
        volatile double dummy_var = 1.0;  // volatile to prevent optimization
        for (long i = 0; i < load; i++) {
            dummy_var *= 1.00001;
        }
    }

    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto start = std::chrono::high_resolution_clock::now();
        dummy_task(load_);  // Add CPU load
        auto stop = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = stop - start;

        RCLCPP_INFO(this->get_logger(), "Elapsed time for dummy_task: %f seconds", elapsed.count());
        
        publisher_->publish(*msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    long load_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc != 5) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Usage: ros2 run <package_name> <executable_name> <node_name> <sub_topic_name> <pub_topic_name> <dummy_task count>");
        return 1;
    }

    std::string node_name = argv[1];
    std::string sub_topic_name = argv[2];
    std::string pub_topic_name = argv[3];
    long load = std::stol(argv[4]);

    rclcpp::spin(std::make_shared<DynamicNode>(node_name, pub_topic_name, sub_topic_name, load));
    rclcpp::shutdown();
    return 0;
}
