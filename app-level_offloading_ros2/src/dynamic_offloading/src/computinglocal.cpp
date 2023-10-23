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

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

//about new_header: header(stamp[rclcpp::Time] + frame_id[string]) + seq[uint32]
#include <ctime>
#include "std_msgs/msg/header.hpp"
#include "header_interfaces/msg/new_header.hpp"
#include "rclcpp/clock.hpp"

// matrix multipliation

#define N 10

int mat[N][N];
int mul[N][N];

void fillRemaining(int i, int j, int n)
{
    // Initialize value to be filled
    int x = 2;
 
    // Fill all values below i as 2, 3, ...p
    for (int k = i + 1; k < n; k++)
        mat[k][j] = x++;
 
    // Fill all values above i
    // as p + 1, p + 2, .. n
    for (int k = 0; k < i; k++)
        mat[k][j] = x++;
}

void constructMatrix(int n)
{
    // Alternatively fill 1s starting from
    // rightmost and leftmost columns. For
    // example for n = 3, we get { {_ _ 1},
    // {1 _ _} {_ 1 _}}
    int right = n - 1, left = 0;
    for (int i = 0; i < n; i++)
    {
        // If i is even, then fill 
        // next column from right
        if (i % 2 == 0)
        {
            mat[i][right] = 1;
 
            // After filling 1, fill remaining
            // entries of column "right"
            fillRemaining(i, right, n);
 
            // Move right one column back
            right--;
        }
         
        // Fill next column from left
        else
        {
            mat[i][left] = 1;
 
            // After filling 1, fill remaining
            // entries of column "left"
            fillRemaining(i, left, n);
 
            // Move left one column forward
            left++;
        }
    }
}

void multiplication(){
for(int i=0;i<N;i++)    
{    
for(int j=0;j<N;j++)    
{    
mul[i][j]=0;    
for(int k=0;k<N;k++)    
{    
mul[i][j]+=mat[i][k]*mat[k][j];    
}    
}    
} 
}

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("computing_node_local")
  {
    /*subscription_ = this->create_subscription<std_msgs::msg::String>(
      "InputData", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));*/
    subscription_2 = this->create_subscription<header_interfaces::msg::NewHeader>(
      "RawData", 10, std::bind(&MinimalSubscriber::topic_callback2, this, _1));
    //publisher_ = this->create_publisher<std_msgs::msg::String>("Result", 10);
    publisher_2 = this->create_publisher<header_interfaces::msg::NewHeader>("Result_Local", 10);
  }

private:

  /*void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    // part of computation > matrix multiple
    multiplication();
    // or sleep
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    
  }*/

  void topic_callback2(const header_interfaces::msg::NewHeader::SharedPtr msg)
  {
    // part of computation > matrix multiple
    multiplication();
    // or sleep
    auto time1 = header_interfaces::msg::NewHeader();
    
    time1.header.stamp = this->now();
    //time1.header.stamp.sec = time1.header.stamp.sec - msg->header.stamp.sec;
    //time1.header.stamp.nanosec = time1.header.stamp.nanosec - msg->header.stamp.nanosec;
    time1.seq = msg->seq;
    publisher_2->publish(time1);
    RCLCPP_INFO(this->get_logger(), "publishing %dth Result, Time to Compute (T1): %dsec %unanosec", time1.seq, time1.header.stamp.sec, time1.header.stamp.nanosec);
  }
  //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<header_interfaces::msg::NewHeader>::SharedPtr subscription_2;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<header_interfaces::msg::NewHeader>::SharedPtr publisher_2;
};

int main(int argc, char * argv[])
{
  constructMatrix(N);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
