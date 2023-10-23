#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <string>
#include <sstream>
#include <vector>

class DynamicNode : public rclcpp::Node
{
public:
    DynamicNode(const std::string & node_name, const std::vector<std::string> & pub_topic_names, const std::vector<std::string> & sub_topic_names, long load, size_t depth)
    : Node(node_name), load_(load)
    {
        rclcpp::QoS qos(depth);

        for (const auto & topic_name : pub_topic_names) {
            publishers_.push_back(this->create_publisher<std_msgs::msg::String>(topic_name, qos));
        }

        for (const auto & topic_name : sub_topic_names) {
            subscriptions_.push_back(
                this->create_subscription<std_msgs::msg::String>(
                    topic_name, qos, std::bind(&DynamicNode::topic_callback, this, std::placeholders::_1))
            );
        }
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
        std::string time = std::to_string(this->now().nanoseconds());

        dummy_task(load_);

        std::string count = extract_count(msg->data);

        std::string output = count + " " + time;
        if (count != "10") std::cout << output << std::endl;

        for (auto & publisher : publishers_) {
            publisher->publish(*msg);
        }
    }

    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
    long load_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 8) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Not enough arguments. Usage: ros2 run node_gen nsub_npub <node_name> <num_of_sub_topics> <sub_topic_name1> <sub_topic_name2> ... <num_of_pub_topics> <pub_topic_name1> <pub_topic_name2> ... <dummy_task count> [qos_depth]");
        return 1;
    }

    std::string node_name = argv[1];
    long load = std::stol(argv[argc-2]);

    size_t depth = 10;  // default QoS depth
    if (argc > 5) {
        depth = std::stoul(argv[argc-1]);
    }

    int num_of_sub_topics = std::stoi(argv[2]);
    int num_of_pub_topics = std::stoi(argv[3+num_of_sub_topics]);

    if(num_of_sub_topics < 1 || num_of_sub_topics > 10 || num_of_pub_topics < 1 || num_of_pub_topics > 10) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Number of topics should be between 1 and 10");
        return 1;
    }

    if(argc != 4 + num_of_sub_topics + num_of_pub_topics + 1 + (argc > 5 ? 1 : 0)) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "The number of provided topic names does not match the expected number of topics.");
        return 1;
    }

    std::vector<std::string> sub_topic_names(&argv[3], &argv[3+num_of_sub_topics]);
    std::vector<std::string> pub_topic_names(&argv[4+num_of_sub_topics], &argv[4+num_of_sub_topics+num_of_pub_topics]);

    rclcpp::spin(std::make_shared<DynamicNode>(node_name, pub_topic_names, sub_topic_names, load, depth));
    rclcpp::shutdown();
    return 0;
}
