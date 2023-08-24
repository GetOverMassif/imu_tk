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

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std;

class MinimalSubscriber : public rclcpp::Node {
  public:
    MinimalSubscriber()
        : Node("minimal_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 100, std::bind(&MinimalSubscriber::topic_callback, this, _1));


        this->get_parameter("limit_time", limit_time);
        this->get_parameter("limit_num", limit_num);
        this->get_parameter("target_path", target_path);

        cout << "target_path = " << target_path << endl;

        RCLCPP_INFO(this->get_logger(), "limit_num: %d", limit_num);
        RCLCPP_INFO(this->get_logger(), "limit_time: %f", limit_time);
        RCLCPP_INFO(this->get_logger(), "target_path: %s", target_path.c_str());

        this->target_file_path_acc = target_path + "/acc_data.txt";
        this->target_file_path_gyro = target_path + "/gyro_data.txt";
    }

  private:
    bool is_first = true;
    double start_time = 0.0;

    double limit_time;
    int limit_num;
    std::string target_path;

    fstream f_acc;
    fstream f_gyro;
    string target_file_path_acc;
    string target_file_path_gyro;

    //   void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (this->is_first) {
            cout << this->is_first << endl;
            this->f_acc.open(this->target_file_path_acc, ios::out);
            this->f_gyro.open(this->target_file_path_gyro, ios::out);
            // this->f << "123323" << " " << "abs" << endl;
            // this->f.close();
            this->is_first = false;
            this->start_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        }

        // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->header.stamp.sec);
        double time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 - this->start_time;

        this->f_acc << setiosflags(ios::scientific) << setprecision(7) << time << "\t";
        this->f_acc << msg->linear_acceleration.x << "\t" << msg->linear_acceleration.y << "\t" << msg->linear_acceleration.z << endl;

        this->f_gyro << setiosflags(ios::scientific) << setprecision(7) << time << "\t";
        this->f_gyro << msg->angular_velocity.x << "\t" << msg->angular_velocity.y << "\t" << msg->angular_velocity.z << endl;

        // cout << "-------------------------------------------------------" << endl;
        // cout << "time: " << msg->header.stamp.sec << "."
        //                  << msg->header.stamp.nanosec << endl;
        // cout << "angular_velocity: " << msg->angular_velocity.x << ", "
        //                              << msg->angular_velocity.y << ", "
        //                              << msg->angular_velocity.z << endl;
        // cout << "linear_acceleration: " << msg->linear_acceleration.x << ", "
        //                                 << msg->linear_acceleration.y << ", "
        //                                 << msg->linear_acceleration.z << endl;
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // if (argc < 2) {
    //     printf("Please follow format: \nros2 run imu_calib imu_data_receiver <target_save_path>");
    // }
    // 目标保存路径、最大数据采集时长、最大采集数据量
    // string target_path = argv[1];

    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
