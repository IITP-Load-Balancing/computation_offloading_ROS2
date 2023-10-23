#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <sstream>

class StepPayloadPublisher : public rclcpp::Node
{
public:
    StepPayloadPublisher(const std::string & node_name, const std::string & topic_name, std::chrono::milliseconds period, long load, std::size_t payload_size, std::size_t num_publishes, std::size_t step_size)
    : Node(node_name), load_(load), max_payload_size_(payload_size), num_publishes_(num_publishes), step_size_(step_size), current_payload_size_(step_size), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
        timer_ = this->create_wall_timer(period, std::bind(&StepPayloadPublisher::timer_callback, this));
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
        dummy_task(load_);

        std_msgs::msg::String message;
        message.data = "Data#" + std::to_string(count_++) + " " + std::string(current_payload_size_, 'a');

        publisher_->publish(message);

        auto now = this->now();
        std::string output = std::to_string(current_payload_size_) + " " + std::to_string(count_) + " " + std::to_string(now.nanoseconds()/1000000);

        std::cout << output << std::endl;

        ++publish_count_;
        if (publish_count_ >= num_publishes_) {
            publish_count_ = 0;
            current_payload_size_ += step_size_;
        }

        if (current_payload_size_ > max_payload_size_) {
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    long load_;
    std::size_t current_payload_size_;
    std::size_t max_payload_size_;
    std::size_t num_publishes_;
    std::size_t step_size_;
    std::size_t count_;
    std::size_t publish_count_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc != 8) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Usage: ros2 run node_gen payload_pub <node_name> <topic_name> <period_millisec> <dummy_task count> <maximum_payload_size> <num_publishes> <step_size>");
        return 1;
    }

    std::string node_name = argv[1];
    std::string topic_name = argv[2];
    std::chrono::milliseconds period(std::stoi(argv[3]));
    long load = std::stol(argv[4]);
    std::size_t payload_size = std::stoi(argv[5]);
    std::size_t num_publishes = std::stoi(argv[6]);
    std::size_t step_size = std::stoi(argv[7]);

    rclcpp::spin(std::make_shared<StepPayloadPublisher>(node_name, topic_name, period, load, payload_size, num_publishes, step_size));

    rclcpp::shutdown();
    return 0;
}
