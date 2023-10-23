#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>

class DynamicPubNode : public rclcpp::Node
{
public:
    DynamicPubNode(const std::string & node_name, const std::string & topic_name, int period_millisec, long load)
    : Node(node_name), load_(load)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name, 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_millisec),
            std::bind(&DynamicPubNode::timer_callback, this));
    }

private:
    void dummy_task(long load) {
        volatile double dummy_var = 1.0;  // volatile to prevent optimization
        for (long i = 0; i < load; i++) {
            dummy_var *= 1.00001;
        }
    }

    void timer_callback()
    {
        auto start = std::chrono::high_resolution_clock::now();
        dummy_task(load_);  // Add CPU load
        auto stop = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = stop - start;

        RCLCPP_INFO(this->get_logger(), "Elapsed time for dummy_task: %f seconds", elapsed.count());

        std_msgs::msg::Float64MultiArray message;
        message.data = generate_random_values(10);  // Generate 10 random values
        publisher_->publish(message);
    }

    std::vector<double> generate_random_values(size_t size)
    {
        std::vector<double> values(size);
        for (auto& value : values) {
            value = static_cast<double>(rand()) / RAND_MAX;
        }
        return values;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    long load_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc != 5) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Usage: ros2 run node_gen dynamic_pub <node_name> <topic_name> <period_millisec> <dummy_task count>");
        return 1;
    }

    std::string node_name = argv[1];
    std::string topic_name = argv[2];
    int period_millisec = std::stoi(argv[3]);
    long load = std::stol(argv[4]);

    rclcpp::spin(std::make_shared<DynamicPubNode>(node_name, topic_name, period_millisec, load));
    rclcpp::shutdown();
    return 0;
}
