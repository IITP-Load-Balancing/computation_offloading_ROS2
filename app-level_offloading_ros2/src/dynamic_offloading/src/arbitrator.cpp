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

#include <memory>
#include <sstream>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
//#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

//about new_header: header(stamp[rclcpp::Time] + frame_id[string]) + seq[uint32]
#include <ctime>
#include "std_msgs/msg/header.hpp"
#include "header_interfaces/msg/new_header.hpp"
#include "rclcpp/clock.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

rclcpp::SyncParametersClient::SharedPtr parameters_client;

int current_seq = 0;
int localcount = 0;
int servercount = 0;

class MinimalSubscriber : public rclcpp::Node 
{
public:
  MinimalSubscriber()
  : Node("arbitrator")
  {
    subscription_ = this->create_subscription<header_interfaces::msg::NewHeader>(
      "Result_Local", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    subscription_2 = this->create_subscription<header_interfaces::msg::NewHeader>(
      "Result_Server", 10, std::bind(&MinimalSubscriber::topic_callback2, this, _1));
    publisher_2 = this->create_publisher<header_interfaces::msg::NewHeader>("Result_Better", 10);
    parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    this->declare_parameter("local_delayed");
    this->declare_parameter("using_server");
    this->declare_parameter("server_delayed");
    this->set_parameters({
    rclcpp::Parameter("local_delayed", 0),
    rclcpp::Parameter("using_server", 0),
    rclcpp::Parameter("server_delayed", 1),
  });
  }

private:

  void topic_callback2(const header_interfaces::msg::NewHeader::SharedPtr msg)
  {
    auto pointer = this -> get_parameter("local_delayed");
    int is_local_delayed = pointer.get_value<int>();
    RCLCPP_INFO(this->get_logger(), "---------------------------use server : %d---------------------------", is_local_delayed);
    if (is_local_delayed == 0) return;

    if (current_seq > msg->seq) return;
    if (current_seq <= msg->seq) current_seq = msg-> seq + 1;
    auto time2 = header_interfaces::msg::NewHeader();
    
    time2.header.stamp = this->now();
    //time2.header.stamp.sec = time2.header.stamp.sec - msg->header.stamp.sec;
    //time2.header.stamp.nanosec = time2.header.stamp.nanosec - msg->header.stamp.nanosec;
    time2.seq = msg->seq;

    publisher_2->publish(time2);
    servercount++;

    RCLCPP_INFO(this->get_logger(), "---------------------------Local: %d, Server: %d ---------------------------", localcount, servercount);
    //RCLCPP_INFO(this->get_logger(), "Server %dth Result, Time to Networking (T2): %dsec %unanosec", time2.seq, time2.header.stamp.sec, time2.header.stamp.nanosec);
  }

  void topic_callback(const header_interfaces::msg::NewHeader::SharedPtr msg)
  {

    auto pointer = this -> get_parameter("server_delayed");
    int is_local_delayed = pointer.get_value<int>();
    RCLCPP_INFO(this->get_logger(), "---------------------------use local : %d---------------------------", is_local_delayed);
    if (is_local_delayed == 0) return;

    if (current_seq > msg->seq) return;
    if (current_seq <= msg->seq) current_seq = msg-> seq + 1;
    auto time2 = header_interfaces::msg::NewHeader();
    
    time2.header.stamp = this->now();
    //time2.header.stamp.sec = time2.header.stamp.sec - msg->header.stamp.sec;
    //time2.header.stamp.nanosec = time2.header.stamp.nanosec - msg->header.stamp.nanosec;
    time2.seq = msg->seq;

    publisher_2->publish(time2);
    localcount++;

    RCLCPP_INFO(this->get_logger(), "---------------------------Local: %d, Server: %d ---------------------------", localcount, servercount);
    //RCLCPP_INFO(this->get_logger(), "Local %dth Result, Time to Networking (T2): %dsec %unanosec", time2.seq, time2.header.stamp.sec, time2.header.stamp.nanosec);
  }

  rclcpp::Subscription<header_interfaces::msg::NewHeader>::SharedPtr subscription_;
  rclcpp::Subscription<header_interfaces::msg::NewHeader>::SharedPtr subscription_2;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<header_interfaces::msg::NewHeader>::SharedPtr publisher_2;
};
/*
  
  //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<header_interfaces::msg::NewHeader>::SharedPtr subscription_2;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<header_interfaces::msg::NewHeader>::SharedPtr publisher_2;
};
*/
int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
