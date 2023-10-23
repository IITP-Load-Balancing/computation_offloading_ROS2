#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <sstream>

class VariablePayloadPublisher : public rclcpp::Node
{
public:
    VariablePayloadPublisher(const std::string & node_name, const std::string & topic_name, std::chrono::milliseconds period, long load, std::size_t payload_size = 70000)
    : Node(node_name), count_(0), load_(load), current_payload_size_(0), max_payload_size_(payload_size)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
        timer_ = this->create_wall_timer(period, std::bind(&VariablePayloadPublisher::timer_callback, this));

        std::cout << "ros2 run node_gen variable_payload_pub <node_name> <topic_name> <period_millisec> <dummy_task count> [maximum_payload_size=70000]" << std::endl;
        std::ostringstream command;
        command << "ros2 run node_gen variable_payload_pub " << node_name << " " << topic_name << " " << period.count() << " " << load << " " << payload_size;
        std::cout << command.str() << std::endl;
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
        message.data = "Data#" + std::to_string(count_) + " " + std::string(current_payload_size_, 'a');
        count_++;
        auto now = this->now();
        publisher_->publish(message);


        std::string output = std::to_string(count_ - 1) + " " + std::to_string(now.nanoseconds()/1000000);

        std::cout << output << std::endl;

        if (current_payload_size_ < max_payload_size_) {
            current_payload_size_ += 1000; //TODO: change this unit
        } else {
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::size_t count_;
    long load_;
    std::size_t current_payload_size_;
    std::size_t max_payload_size_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 5 || argc > 6) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Usage: ros2 run node_gen variable_payload_pub <node_name> <topic_name> <period_millisec> <dummy_task count> [maximum_payload_size=70000]");
        return 1;
    }

    std::string node_name = argv[1];
    std::string topic_name = argv[2];
    std::chrono::milliseconds period(std::stoi(argv[3]));
    long load = std::stol(argv[4]);
    std::size_t payload_size = (argc == 6) ? std::stoi(argv[5]) : 70000;

    rclcpp::spin(std::make_shared<VariablePayloadPublisher>(node_name, topic_name, period, load, payload_size));
    return 0;
}
