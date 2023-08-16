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
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;
using namespace std;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 100, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->header.stamp.sec);
    cout << "-------------------------------------------------------" << endl;
    cout << "time: " << msg->header.stamp.sec << "." 
                     << msg->header.stamp.nanosec << endl;
    cout << "angular_velocity: " << msg->angular_velocity.x << ", "
                                 << msg->angular_velocity.y << ", "
                                 << msg->angular_velocity.z << endl;
    cout << "linear_acceleration: " << msg->linear_acceleration.x << ", "
                                    << msg->linear_acceleration.y << ", "
                                    << msg->linear_acceleration.z << endl;
  }
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
