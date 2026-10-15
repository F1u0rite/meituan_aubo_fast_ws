#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

using FollowJT = control_msgs::action::FollowJointTrajectory;
using ListHardware =
    controller_manager_msgs::srv::ListHardwareComponents;

const std::array<std::string, 6> JOINT_NAMES = {
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint"
};

int run(const rclcpp::Node::SharedPtr &node)
{
    // 1. 查询硬件插件，严格限制为 Fake Hardware
    auto hw_client = node->create_client<ListHardware>(
        "/controller_manager/list_hardware_components");

    if (!hw_client->wait_for_service(3s)) {
        RCLCPP_ERROR(node->get_logger(), "Hardware service unavailable");
        return 1;
    }

    auto hw_future =
        hw_client->async_send_request(
            std::make_shared<ListHardware::Request>());

    if (rclcpp::spin_until_future_complete(
            node, hw_future, 3s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Hardware query failed");
        return 1;
    }

    const auto components = hw_future.get()->component;

    if (components.size() != 1 ||
        components[0].name != "auboHardwareInterface" ||
        components[0].plugin_name != "mock_components/GenericSystem" ||
        components[0].state.label != "active") {
        RCLCPP_ERROR(
            node->get_logger(),
            "NOT Fake Hardware. Refusing to execute.");
        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Fake Hardware verified. Motion test permitted.");

    // 2. 订阅当前六关节位置
    std::array<double, 6> current{};
    bool state_valid = false;

    auto last_state = std::chrono::steady_clock::time_point{};

    auto subscription =
        node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            [&](sensor_msgs::msg::JointState::SharedPtr msg)
            {
                std::array<double, 6> values{};

                for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
                    auto it = std::find(
                        msg->name.begin(),
                        msg->name.end(),
                        JOINT_NAMES[i]);

                    if (it == msg->name.end()) {
                        return;
                    }

                    const size_t index =
                        std::distance(msg->name.begin(), it);

                    if (index >= msg->position.size() ||
                        !std::isfinite(msg->position[index])) {
                        return;
                    }

                    values[i] = msg->position[index];
                }

                current = values;
                state_valid = true;
                last_state = std::chrono::steady_clock::now();
            });

    const auto state_deadline =
        std::chrono::steady_clock::now() + 3s;

    while (rclcpp::ok() &&
           std::chrono::steady_clock::now() < state_deadline) {
        rclcpp::spin_some(node);

        if (state_valid &&
            std::chrono::steady_clock::now() - last_state < 1s) {
            break;
        }

        std::this_thread::sleep_for(50ms);
    }

    if (!state_valid ||
        std::chrono::steady_clock::now() - last_state >= 1s) {
        RCLCPP_ERROR(node->get_logger(), "No fresh joint states");
        return 1;
    }

    // 3. 创建 FollowJointTrajectory Action Client
    auto client = rclcpp_action::create_client<FollowJT>(
        node,
        "/joint_trajectory_controller/follow_joint_trajectory");

    if (!client->wait_for_action_server(3s)) {
        RCLCPP_ERROR(node->get_logger(), "Action server unavailable");
        return 1;
    }

    // 4. 构造测试轨迹
    // 仅用于 Fake Hardware：J1 相对当前位置移动 0.02 rad
    std::vector<double> start(current.begin(), current.end());
    std::vector<double> target = start;

    // Fake Hardware only: non-singular IK test configuration.
    // Keep J1, J4 and J6 at their current positions.
    target[1] = -0.30;  // J2
    target[2] =  0.60;  // J3
    target[4] =  0.30;  // J5


    FollowJT::Goal goal;

    goal.trajectory.joint_names.assign(
        JOINT_NAMES.begin(), JOINT_NAMES.end());

    trajectory_msgs::msg::JointTrajectoryPoint p0;
    p0.positions = start;
    p0.velocities.assign(6, 0.0);
    p0.time_from_start =
        rclcpp::Duration::from_seconds(0.5);

    trajectory_msgs::msg::JointTrajectoryPoint p1;
    p1.positions = target;
    p1.velocities.assign(6, 0.0);
    p1.time_from_start =
        rclcpp::Duration::from_seconds(2.5);

    goal.trajectory.points = {p0, p1};

    RCLCPP_INFO(
        node->get_logger(),
        "Mock J1: %.5f -> %.5f rad",
        start[0], target[0]);

    // 5. 接收执行反馈
    rclcpp_action::Client<FollowJT>::SendGoalOptions options;

    options.feedback_callback =
        [node](
            auto,
            const std::shared_ptr<const FollowJT::Feedback> feedback)
        {
            if (!feedback->actual.positions.empty()) {
                RCLCPP_INFO_THROTTLE(
                    node->get_logger(),
                    *node->get_clock(),
                    500,
                    "Feedback J1: %.5f rad",
                    feedback->actual.positions[0]);
            }
        };

    // 6. 发送 Goal
    auto goal_future = client->async_send_goal(goal, options);

    if (rclcpp::spin_until_future_complete(
            node, goal_future, 5s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Goal request timed out");
        return 1;
    }

    auto goal_handle = goal_future.get();

    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Goal rejected");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Goal accepted");

    // 7. 等待最终执行结果
    auto result_future = client->async_get_result(goal_handle);

    if (rclcpp::spin_until_future_complete(
            node, result_future, 10s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Result timeout; execution state must be checked");
        return 1;
    }

    const auto result = result_future.get();

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !result.result ||
        result.result->error_code != FollowJT::Result::SUCCESSFUL) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Trajectory failed. Action code: %d",
            static_cast<int>(result.code));
        return 1;
    }

    // 8. 独立检查最终关节位置
    const auto verify_deadline =
        std::chrono::steady_clock::now() + 2s;

    while (rclcpp::ok() &&
           std::chrono::steady_clock::now() < verify_deadline) {
        rclcpp::spin_some(node);

        if (std::chrono::steady_clock::now() - last_state < 1s) {
            bool reached = true;

            for (size_t i = 0; i < 6; ++i) {
                if (std::fabs(current[i] - target[i]) > 0.01) {
                    reached = false;
                    break;
                }
            }

            if (reached) {
                RCLCPP_INFO(
                    node->get_logger(),
                    "SUCCESS: Mock trajectory completed and verified");
                return 0;
            }
        }

        std::this_thread::sleep_for(50ms);
    }

    RCLCPP_ERROR(
        node->get_logger(),
        "Action succeeded, but final joint state verification failed");

    return 1;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "mtc_trajectory_test");

    const int result = run(node);

    rclcpp::shutdown();
    return result;
}