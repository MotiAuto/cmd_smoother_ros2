#include "cmd_smoother_ros2.hpp"

namespace cmd_smoother_ros2
{
    CmdSmootherROS2::CmdSmootherROS2(const rclcpp::NodeOptions & node_options):Node("vel_sm_ros2_node", node_options)
    {
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            0,
            std::bind(&cmd_smoother_ros2::CmdSmootherROS2::topic_callback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/smoothed", 0);

        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&cmd_smoother_ros2::CmdSmootherROS2::timer_callback, this));

        this->declare_parameter("up_gain", 0.3);
        this->get_parameter("up_gain", up_gain_);
        this->declare_parameter("down_gain", 0.1);
        this->get_parameter("down_gain", down_gain_);
        this->declare_parameter("enable_debug", true);
        this->get_parameter("enable_debug", enable_);

        target_ = geometry_msgs::msg::Twist();
        now_ = geometry_msgs::msg::Twist();

        if(up_gain_ > down_gain_)
        {
            gain = down_gain_;
        }
        else
        {
            gain = up_gain_;
        }

        RCLCPP_INFO(this->get_logger(), "Start VelSmoothROS2");
    }

    void CmdSmootherROS2::topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_ = *msg;
    }

    void CmdSmootherROS2::timer_callback()
    {
        auto send = geometry_msgs::msg::Twist();
        auto vec = target_.linear.x - now_.linear.x;
        vec = std::sqrt(vec*vec);

        if(vec > gain)
        {
            if(target_.linear.x > now_.linear.x)
            {
                if(now_.linear.x < 0.0)
                {
                    send.linear.x = now_.linear.x + down_gain_;
                }
                else
                {
                    send.linear.x = now_.linear.x + up_gain_;
                }
            }
            else
            {
                if(target_.linear.x < 0.0)
                {
                    send.linear.x = now_.linear.x - up_gain_;
                }
                else
                {
                    send.linear.x = now_.linear.x - down_gain_;
                }
            }
        }
        else
        {
            send.linear.x = target_.linear.x;
        }
        now_.linear.x = send.linear.x;

        vec = target_.linear.y - now_.linear.y;
        vec = std::sqrt(vec*vec);

        if(vec > gain)
        {
            if(target_.linear.y > now_.linear.y)
            {
                if(now_.linear.y < 0.0)
                {
                    send.linear.y = now_.linear.y + down_gain_;
                }
                else
                {
                    send.linear.y = now_.linear.y + up_gain_;
                }
            }
            else
            {
                if(target_.linear.y < 0.0)
                {
                    send.linear.y = now_.linear.y - up_gain_;
                }
                else
                {
                    send.linear.y = now_.linear.y - down_gain_;
                }
            }
        }
        else
        {
            send.linear.y = target_.linear.y;
        }

        now_.linear.y = send.linear.y;

        vec = target_.angular.z - now_.angular.z;
        vec = std::sqrt(vec*vec);

        if(vec > gain)
        {
            if(target_.angular.z > now_.angular.z)
            {
                if(now_.angular.z < 0.0)
                {
                    send.angular.z = now_.angular.z + down_gain_;
                }
                else
                {
                    send.angular.z = now_.angular.z+ up_gain_;
                }
            }
            else
            {
                if(target_.angular.z < 0.0)
                {
                    send.angular.z = now_.angular.z - up_gain_;
                }
                else
                {
                    send.angular.z = now_.angular.z - down_gain_;
                }
            }
        }
        else
        {
            send.angular.z = target_.angular.z;
        }

        now_.angular.z = send.angular.z;

        if(enable_)
        {
            RCLCPP_INFO(this->get_logger(), "x:%.2lf, y:%.2lf, z:%.2lf", send.linear.x, send.linear.y, send.angular.z);
        }

        publisher_->publish(send);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cmd_smoother_ros2::CmdSmootherROS2)