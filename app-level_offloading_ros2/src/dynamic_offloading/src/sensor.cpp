// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

//about new_header: header(stamp[rclcpp::Time] + frame_id[string]) + seq[uint32]
#include "std_msgs/msg/header.hpp"
#include "header_interfaces/msg/new_header.hpp"
#include "rclcpp/clock.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("sensor"), count_(0)
  {
    //publisher_ = this->create_publisher<std_msgs::msg::String>("InputData", 10);
    publisher_2 = this->create_publisher<header_interfaces::msg::NewHeader>("RawData", 10);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    /*
    auto message = std_msgs::msg::String();
    message.data = std::to_string(++count_);
    RCLCPP_INFO(this->get_logger(), "_________________________________", message.data.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing %sth InputData", message.data.c_str());
    publisher_->publish(message);
    */

    auto time0 = header_interfaces::msg::NewHeader();
    time0.header.stamp = this->now();
    time0.seq = ++count_;
    time0.value.data.push_back(count_);
    RCLCPP_INFO(this->get_logger(), "publishing %dth RawData, Start Time (T0): %dsec %unanosec", time0.seq, time0.header.stamp.sec, time0.header.stamp.nanosec);
    publisher_2->publish(time0);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<header_interfaces::msg::NewHeader>::SharedPtr publisher_2;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
