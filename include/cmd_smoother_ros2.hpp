#ifndef CMD_SMOOTHER_ROS2_HPP_
#define CMD_SMOOTHER_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>

#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace cmd_smoother_ros2
{
    class CmdSmootherROS2 : public rclcpp::Node
    {
        public:
        CmdSmootherROS2(const rclcpp::NodeOptions & node_options=rclcpp::NodeOptions());

        private:
        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void float_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void twist_timer_callback();
        void float_timer_callback();

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_float_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr float_publisher_;
        float up_gain_, down_gain_, gain;
        std::string msg_type_;
        bool enable_;
        geometry_msgs::msg::Twist target_twist_;
        geometry_msgs::msg::Twist now_twist_;
        std_msgs::msg::Float32 target_float_;
        std_msgs::msg::Float32 now_float_;
    };
}

#endif