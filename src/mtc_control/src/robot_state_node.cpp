#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>

class RobotStateNode : public rclcpp::Node
{
public:
    RobotStateNode()
        : Node("robot_state_node")
    {
        subscription_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",
                rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::JointState::SharedPtr msg)
                {
                    joint_callback(msg);
                });

        RCLCPP_INFO(
            this->get_logger(),
            "AUBO S3 Robot State Node started."
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for /joint_states ..."
        );
    }

private:
    void joint_callback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        const std::array<std::string, 6> joint_names = {
            "shoulder_joint",
            "upperArm_joint",
            "foreArm_joint",
            "wrist1_joint",
            "wrist2_joint",
            "wrist3_joint"
        };

        std::ostringstream output;
        output << std::fixed << std::setprecision(4);

        for (const auto &joint : joint_names)
        {
            auto it = std::find(
                msg->name.begin(),
                msg->name.end(),
                joint
            );

            if (it == msg->name.end())
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Missing joint: %s",
                    joint.c_str()
                );
                return;
            }

            size_t index = std::distance(
                msg->name.begin(), it
            );

            if (index >= msg->position.size())
            {
                return;
            }

            output << joint
                   << ": "
                   << msg->position[index]
                   << " rad";

            if (index < msg->velocity.size())
            {
                output << "  vel="
                       << msg->velocity[index]
                       << " rad/s";
            }

            output << "\n";
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "\n%s",
            output.str().c_str()
        );
    }

    rclcpp::Subscription<
        sensor_msgs::msg::JointState
    >::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotStateNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}