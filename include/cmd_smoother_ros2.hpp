#ifndef CMD_SMOOTHER_ROS2_HPP_
#define CMD_SMOOTHER_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace cmd_smoother_ros2
{
    class CmdSmootherROS2 : public rclcpp::Node
    {
        public:
        CmdSmootherROS2(const rclcpp::NodeOptions & node_options=rclcpp::NodeOptions());

        private:
        void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void timer_callback();

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        float up_gain_, down_gain_, gain;
        bool enable_;
        geometry_msgs::msg::Twist target_;
        geometry_msgs::msg::Twist now_;
    };
}

#endif