#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
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
using ListHW = controller_manager_msgs::srv::ListHardwareComponents;

using Trajectory = trajectory_msgs::msg::JointTrajectory;
using JointState = sensor_msgs::msg::JointState;

static const std::array<std::string, 6> JOINTS = {
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint"
};

// AUBO S3 URDF limits.
// 本节点专用于当前 S3 Mock 测试。
static const std::array<double, 6> LOWER = {
    -6.283185307179586,
    -3.141592653589793,
    -2.87979326579064,
    -6.283185307179586,
    -6.283185307179586,
    -6.283185307179586
};

static const std::array<double, 6> UPPER = {
    6.283185307179586,
    3.141592653589793,
    2.87979326579064,
    6.283185307179586,
    6.283185307179586,
    6.283185307179586
};

static double point_time(
    const trajectory_msgs::msg::JointTrajectoryPoint &p)
{
    return static_cast<double>(p.time_from_start.sec) +
           static_cast<double>(p.time_from_start.nanosec) * 1e-9;
}

static bool validate_trajectory(
    const Trajectory &trajectory,
    const std::array<double, 6> &current,
    const rclcpp::Logger &logger)
{
    if (trajectory.joint_names.size() != 6 ||
        !std::equal(
            JOINTS.begin(),
            JOINTS.end(),
            trajectory.joint_names.begin())) {
        RCLCPP_ERROR(logger, "Unexpected joint names/order");
        return false;
    }

    const auto &points = trajectory.points;

    if (points.size() < 6 || points.size() > 201) {
        RCLCPP_ERROR(logger, "Invalid trajectory point count");
        return false;
    }

    if (std::abs(point_time(points.front())) > 1e-9) {
        RCLCPP_ERROR(logger, "First point must start at t=0");
        return false;
    }

    const double total_time = point_time(points.back());

    if (!std::isfinite(total_time) ||
        total_time < 2.0 || total_time > 10.0) {
        RCLCPP_ERROR(logger, "Invalid trajectory duration");
        return false;
    }

    for (size_t i = 0; i < points.size(); ++i) {
        const auto &p = points[i];

        if (p.positions.size() != 6 ||
            !p.velocities.empty() ||
            !p.accelerations.empty() ||
            !p.effort.empty()) {
            RCLCPP_ERROR(
                logger,
                "Unsupported point format at %zu", i);
            return false;
        }

        for (size_t j = 0; j < 6; ++j) {
            const double q = p.positions[j];

            if (!std::isfinite(q) ||
                q < LOWER[j] + 0.01 ||
                q > UPPER[j] - 0.01) {
                RCLCPP_ERROR(
                    logger,
                    "Joint limit check failed at point %zu", i);
                return false;
            }

            // 本次 Mock 验证只允许从起点产生有限关节位移。
            if (std::abs(q - points.front().positions[j]) > 0.15) {
                RCLCPP_ERROR(
                    logger,
                    "Trajectory exceeds mock-test joint range");
                return false;
            }

            if (i == 0) {
                if (std::abs(q - current[j]) > 0.005) {
                    RCLCPP_ERROR(
                        logger,
                        "Start state mismatch: %s",
                        JOINTS[j].c_str());
                    return false;
                }
            } else {
                const double dt =
                    point_time(p) - point_time(points[i - 1]);

                if (!std::isfinite(dt) || dt <= 0.0) {
                    RCLCPP_ERROR(
                        logger,
                        "Invalid time at point %zu", i);
                    return false;
                }

                const double dq =
                    std::abs(q - points[i - 1].positions[j]);

                if (dq > 0.03 || dq / dt > 0.10) {
                    RCLCPP_ERROR(
                        logger,
                        "Joint continuity/speed check failed "
                        "at point %zu", i);
                    return false;
                }
            }
        }
    }

    RCLCPP_INFO(
        logger,
        "Trajectory validated: %zu points, %.3f s",
        points.size(), total_time);

    return true;
}

int run(const rclcpp::Node::SharedPtr &node)
{
    const bool enabled =
        node->declare_parameter<bool>("enable_mock_motion", false);

    if (!enabled) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Motion disabled. Explicitly set "
            "enable_mock_motion:=true for Fake Hardware.");
        return 1;
    }

    // ========================================================
    // 1. Verify Fake Hardware
    // ========================================================

    auto hw_client = node->create_client<ListHW>(
        "/controller_manager/list_hardware_components");

    if (!hw_client->wait_for_service(5s)) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Hardware service unavailable");
        return 1;
    }

    auto hw_future = hw_client->async_send_request(
        std::make_shared<ListHW::Request>());

    if (rclcpp::spin_until_future_complete(
            node, hw_future, 5s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Hardware query failed");
        return 1;
    }

    const auto components = hw_future.get()->component;

    if (components.size() != 1 ||
        components[0].name != "auboHardwareInterface" ||
        components[0].plugin_name != "mock_components/GenericSystem" ||
        components[0].state.label != "active") {
        RCLCPP_ERROR(
            node->get_logger(),
            "REAL OR UNKNOWN HARDWARE: execution refused");
        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Fake Hardware verified");

    // ========================================================
    // 2. Subscribe to joint states
    // ========================================================

    std::array<double, 6> current{};
    bool state_valid = false;

    auto last_state = std::chrono::steady_clock::time_point{};

    auto joint_sub =
        node->create_subscription<JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            [&](JointState::SharedPtr msg)
            {
                std::array<double, 6> q{};

                for (size_t i = 0; i < 6; ++i) {
                    auto it = std::find(
                        msg->name.begin(),
                        msg->name.end(),
                        JOINTS[i]);

                    if (it == msg->name.end())
                        return;

                    const size_t index =
                        std::distance(msg->name.begin(), it);

                    if (index >= msg->position.size() ||
                        !std::isfinite(msg->position[index]))
                        return;

                    q[i] = msg->position[index];
                }

                current = q;
                state_valid = true;
                last_state = std::chrono::steady_clock::now();
            });

    // ========================================================
    // 3. Subscribe to Cartesian preview
    // ========================================================

    Trajectory::SharedPtr preview;

    auto preview_sub =
        node->create_subscription<Trajectory>(
            "/mtc/cartesian_preview",
            rclcpp::QoS(1).reliable().transient_local(),
            [&](Trajectory::SharedPtr msg)
            {
                if (msg->header.stamp.sec == 0)
                    return;

                const double age =
                    (node->now() -
                     rclcpp::Time(msg->header.stamp)).seconds();

                // 丢弃旧规划，避免误执行之前保留的预览。
                if (age < -2.0 || age > 60.0) {
                    RCLCPP_WARN(
                        node->get_logger(),
                        "Ignoring stale preview (age=%.1f s)",
                        age);
                    return;
                }

                preview = msg;

                RCLCPP_INFO(
                    node->get_logger(),
                    "Fresh Cartesian preview received");
            });

    RCLCPP_INFO(
        node->get_logger(),
        "Waiting for fresh /mtc/cartesian_preview "
        "and /joint_states ...");

    const auto deadline =
        std::chrono::steady_clock::now() + 45s;

    while (rclcpp::ok() &&
           std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node);

        const bool state_fresh =
            state_valid &&
            std::chrono::steady_clock::now() - last_state < 1s;

        if (preview && state_fresh)
            break;

        std::this_thread::sleep_for(30ms);
    }

    if (!preview || !state_valid ||
        std::chrono::steady_clock::now() - last_state >= 1s) {
        RCLCPP_ERROR(
            node->get_logger(),
            "No fresh preview or joint state");
        return 1;
    }

    // ========================================================
    // 4. Validate preview
    // ========================================================

    if (!validate_trajectory(
            *preview, current, node->get_logger())) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Preview validation failed");
        return 1;
    }

    // ========================================================
    // 5. Create FollowJointTrajectory Action Client
    // ========================================================

    auto client = rclcpp_action::create_client<FollowJT>(
        node,
        "/joint_trajectory_controller/follow_joint_trajectory");

    if (!client->wait_for_action_server(5s)) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Trajectory Action Server unavailable");
        return 1;
    }

    // 发送前再次确认当前状态与起点匹配。
    rclcpp::spin_some(node);

    if (std::chrono::steady_clock::now() - last_state >= 1s ||
        !validate_trajectory(
            *preview, current, node->get_logger())) {
        RCLCPP_ERROR(
            node->get_logger(),
            "State changed before sending trajectory");
        return 1;
    }

    FollowJT::Goal goal;

    goal.trajectory = *preview;

    // 关键：旧预览的时间戳不能直接交给控制器。
    // 时间戳清零表示从接收时开始执行。
    goal.trajectory.header.stamp.sec = 0;
    goal.trajectory.header.stamp.nanosec = 0;

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
                    "Executing... J1=%.6f rad",
                    feedback->actual.positions[0]);
            }
        };

    RCLCPP_WARN(
        node->get_logger(),
        "Sending Cartesian trajectory to MOCK controller");

    auto goal_future =
        client->async_send_goal(goal, options);

    if (rclcpp::spin_until_future_complete(
            node, goal_future, 5s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Goal request timed out");
        return 1;
    }

    auto goal_handle = goal_future.get();

    if (!goal_handle) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Trajectory goal rejected");
        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Trajectory goal accepted");

    // ========================================================
    // 6. Wait for result
    // ========================================================

    auto result_future =
        client->async_get_result(goal_handle);

    if (rclcpp::spin_until_future_complete(
            node, result_future, 20s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Execution timeout; requesting goal cancellation");

        client->async_cancel_goal(goal_handle);
        rclcpp::spin_some(node);

        return 1;
    }

    const auto result = result_future.get();

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !result.result ||
        result.result->error_code != FollowJT::Result::SUCCESSFUL) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Trajectory execution failed");
        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Action reported SUCCESS");

    // ========================================================
    // 7. Verify final joint position
    // ========================================================

    const auto &last_point =
        preview->points.back();

    const auto verify_deadline =
        std::chrono::steady_clock::now() + 2s;

    while (rclcpp::ok() &&
           std::chrono::steady_clock::now() < verify_deadline) {
        rclcpp::spin_some(node);

        if (std::chrono::steady_clock::now() - last_state < 1s) {
            bool reached = true;
            double max_error = 0.0;

            for (size_t j = 0; j < 6; ++j) {
                const double error = std::abs(
                    current[j] - last_point.positions[j]);

                max_error = std::max(max_error, error);

                if (error > 0.005)
                    reached = false;
            }

            if (reached) {
                RCLCPP_INFO(
                    node->get_logger(),
                    "Final max joint error: %.6f rad",
                    max_error);

                RCLCPP_INFO(
                    node->get_logger(),
                    "SUCCESS: Cartesian preview executed "
                    "in Fake Hardware");

                return 0;
            }
        }

        std::this_thread::sleep_for(30ms);
    }

    RCLCPP_ERROR(
        node->get_logger(),
        "Final joint-state verification failed");

    return 1;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<rclcpp::Node>(
            "mtc_cartesian_executor");

    const int result = run(node);

    rclcpp::shutdown();

    return result;
}