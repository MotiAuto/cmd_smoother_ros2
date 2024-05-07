#include "cmd_smoother_ros2.hpp"

namespace cmd_smoother_ros2
{
    CmdSmootherROS2::CmdSmootherROS2(const rclcpp::NodeOptions & node_options):Node("cmd_smoother_ros2_node", node_options)
    {
        this->declare_parameter("up_gain", 0.3);
        this->get_parameter("up_gain", up_gain_);
        this->declare_parameter("down_gain", 0.1);
        this->get_parameter("down_gain", down_gain_);
        this->declare_parameter("enable_debug", true);
        this->get_parameter("enable_debug", enable_);
        this->declare_parameter("msg_type", "twist");
        this->get_parameter("msg_type", msg_type_);

        target_twist_ = geometry_msgs::msg::Twist();
        now_twist_ = geometry_msgs::msg::Twist();

        if(up_gain_ > down_gain_)
        {
            gain = down_gain_;
        }
        else
        {
            gain = up_gain_;
        }

        if(msg_type_ == "twist")
        {
            sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                0,
                std::bind(&CmdSmootherROS2::twist_callback, this, _1));

            twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/smoothed", 0);

            timer_ = this->create_wall_timer(
            50ms,
            std::bind(&cmd_smoother_ros2::CmdSmootherROS2::twist_timer_callback, this));
        }
        else if(msg_type_ == "float")
        {
            sub_float_ = this->create_subscription<std_msgs::msg::Float32>(
                "/float",
                0,
                std::bind(&CmdSmootherROS2::float_callback, this, _1));

            float_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/smoothed", 0);

            timer_ = this->create_wall_timer(
                50ms,
                std::bind(&CmdSmootherROS2::float_timer_callback, this));
        }

        RCLCPP_INFO(this->get_logger(), "Start VelSmoothROS2");
    }

    void CmdSmootherROS2::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_twist_ = *msg;
    }

    void CmdSmootherROS2::float_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        target_float_ = *msg;
    }

    void CmdSmootherROS2::twist_timer_callback()
    {
        auto send = geometry_msgs::msg::Twist();
        auto vec = target_twist_.linear.x - now_twist_.linear.x;
        vec = std::sqrt(vec*vec);

        if(vec > gain)
        {
            if(target_twist_.linear.x > now_twist_.linear.x)
            {
                if(now_twist_.linear.x < 0.0)
                {
                    send.linear.x = now_twist_.linear.x + down_gain_;
                }
                else
                {
                    send.linear.x = now_twist_.linear.x + up_gain_;
                }
            }
            else
            {
                if(target_twist_.linear.x < 0.0)
                {
                    send.linear.x = now_twist_.linear.x - up_gain_;
                }
                else
                {
                    send.linear.x = now_twist_.linear.x - down_gain_;
                }
            }
        }
        else
        {
            send.linear.x = target_twist_.linear.x;
        }
        now_twist_.linear.x = send.linear.x;

        vec = target_twist_.linear.y - now_twist_.linear.y;
        vec = std::sqrt(vec*vec);

        if(vec > gain)
        {
            if(target_twist_.linear.y > now_twist_.linear.y)
            {
                if(now_twist_.linear.y < 0.0)
                {
                    send.linear.y = now_twist_.linear.y + down_gain_;
                }
                else
                {
                    send.linear.y = now_twist_.linear.y + up_gain_;
                }
            }
            else
            {
                if(target_twist_.linear.y < 0.0)
                {
                    send.linear.y = now_twist_.linear.y - up_gain_;
                }
                else
                {
                    send.linear.y = now_twist_.linear.y - down_gain_;
                }
            }
        }
        else
        {
            send.linear.y = target_twist_.linear.y;
        }

        now_twist_.linear.y = send.linear.y;

        vec = target_twist_.angular.z - now_twist_.angular.z;
        vec = std::sqrt(vec*vec);

        if(vec > gain)
        {
            if(target_twist_.angular.z > now_twist_.angular.z)
            {
                if(now_twist_.angular.z < 0.0)
                {
                    send.angular.z = now_twist_.angular.z + down_gain_;
                }
                else
                {
                    send.angular.z = now_twist_.angular.z+ up_gain_;
                }
            }
            else
            {
                if(target_twist_.angular.z < 0.0)
                {
                    send.angular.z = now_twist_.angular.z - up_gain_;
                }
                else
                {
                    send.angular.z = now_twist_.angular.z - down_gain_;
                }
            }
        }
        else
        {
            send.angular.z = target_twist_.angular.z;
        }

        now_twist_.angular.z = send.angular.z;

        if(enable_)
        {
            RCLCPP_INFO(this->get_logger(), "x:%.2lf, y:%.2lf, z:%.2lf", send.linear.x, send.linear.y, send.angular.z);
        }

        twist_publisher_->publish(send);
    }

    void CmdSmootherROS2::float_timer_callback()
    {
        auto send = std_msgs::msg::Float32();
        auto vec = target_float_.data - now_float_.data;
        vec = std::sqrt(vec*vec);

        if(vec > gain)
        {
            if(target_float_.data > now_float_.data)
            {
                if(now_float_.data < 0.0)
                {
                    send.data = now_float_.data + down_gain_;
                }
                else
                {
                    send.data = now_float_.data + up_gain_;
                }
            }
            else
            {
                if(target_float_.data < 0.0)
                {
                    send.data = now_float_.data - up_gain_;
                }
                else
                {
                    send.data = now_float_.data - down_gain_;
                }
            }
        }
        else
        {
            send.data = target_float_.data;
        }
        now_twist_.linear.x = send.data;

        float_publisher_->publish(send);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cmd_smoother_ros2::CmdSmootherROS2)
