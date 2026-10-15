#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

using namespace std::chrono_literals;

using JointState = sensor_msgs::msg::JointState;
using Trajectory = trajectory_msgs::msg::JointTrajectory;

static const std::array<std::string, 6> JOINTS = {
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint"
};

class CartesianMonitor : public rclcpp::Node
{
public:
    CartesianMonitor() : Node("mtc_cartesian_monitor")
    {
        monitor_start_ns_ = now().nanoseconds();

        urdf_sub_ = create_subscription<std_msgs::msg::String>(
            "/robot_description",
            rclcpp::QoS(1).reliable().transient_local(),
            [this](std_msgs::msg::String::SharedPtr msg) {
                on_urdf(msg);
            });

        joint_sub_ = create_subscription<JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            [this](JointState::SharedPtr msg) {
                on_joint_state(msg);
            });

        preview_sub_ = create_subscription<Trajectory>(
            "/mtc/cartesian_preview",
            rclcpp::QoS(1).reliable().transient_local(),
            [this](Trajectory::SharedPtr msg) {
                on_preview(msg);
            });

        timer_ = create_wall_timer(
            200ms, [this]() { tick(); });

        RCLCPP_INFO(
            get_logger(),
            "Cartesian path monitor started (READ ONLY)");

        RCLCPP_INFO(
            get_logger(),
            "Waiting for a NEW trajectory preview...");
    }

    bool passed() const
    {
        return passed_;
    }

private:
    void on_urdf(const std_msgs::msg::String::SharedPtr msg)
    {
        if (model_ready_) {
            return;
        }

        KDL::Tree tree;

        if (!kdl_parser::treeFromString(msg->data, tree) ||
            !tree.getChain("base_link", "ee_link", chain_)) {
            RCLCPP_ERROR(get_logger(), "Failed to load KDL chain");
            return;
        }

        if (chain_.getNrOfJoints() != 6) {
            RCLCPP_ERROR(get_logger(), "Expected six joints");
            return;
        }

        size_t index = 0;

        for (unsigned int i = 0;
             i < chain_.getNrOfSegments(); ++i) {

            const auto &joint =
                chain_.getSegment(i).getJoint();

            if (joint.getType() == KDL::Joint::None) {
                continue;
            }

            if (index >= 6 ||
                joint.getName() != JOINTS[index]) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Unexpected KDL joint order");
                return;
            }

            ++index;
        }

        fk_ = std::make_unique<
            KDL::ChainFkSolverPos_recursive>(chain_);

        model_ready_ = true;

        RCLCPP_INFO(get_logger(), "KDL model ready");
    }

    bool compute_fk(
        const std::array<double, 6> &positions,
        KDL::Frame &pose)
    {
        if (!fk_) {
            return false;
        }

        KDL::JntArray q(6);

        for (size_t i = 0; i < 6; ++i) {
            q(i) = positions[i];
        }

        return fk_->JntToCart(q, pose) >= 0;
    }

    void on_preview(const Trajectory::SharedPtr msg)
    {
        if (armed_ || done_) {
            return;
        }

        const int64_t stamp_ns =
            static_cast<int64_t>(msg->header.stamp.sec) *
                1000000000LL +
            msg->header.stamp.nanosec;

        // 只接受监测器启动之后生成的规划结果。
        if (stamp_ns < monitor_start_ns_ ||
            stamp_ns > now().nanoseconds() + 2000000000LL) {
            return;
        }

        if (msg->joint_names.size() != 6 ||
            !std::equal(
                JOINTS.begin(), JOINTS.end(),
                msg->joint_names.begin()) ||
            msg->points.size() < 6 ||
            msg->points.size() > 201) {
            RCLCPP_ERROR(
                get_logger(), "Invalid preview trajectory");
            return;
        }

        for (const auto &point : msg->points) {
            if (point.positions.size() != 6) {
                RCLCPP_ERROR(
                    get_logger(), "Invalid trajectory point");
                return;
            }

            for (double q : point.positions) {
                if (!std::isfinite(q)) {
                    RCLCPP_ERROR(
                        get_logger(), "Non-finite joint value");
                    return;
                }
            }
        }

        pending_preview_ = msg;

        RCLCPP_INFO(
            get_logger(), "New trajectory preview received");
    }

    void on_joint_state(const JointState::SharedPtr msg)
    {
        std::array<double, 6> values{};

        for (size_t i = 0; i < 6; ++i) {
            auto it = std::find(
                msg->name.begin(), msg->name.end(), JOINTS[i]);

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

        current_q_ = values;
        state_ready_ = true;
        last_state_time_ = std::chrono::steady_clock::now();

        if (!armed_ || done_) {
            return;
        }

        double max_joint_difference = 0.0;

        for (size_t i = 0; i < 6; ++i) {
            max_joint_difference = std::max(
                max_joint_difference,
                std::abs(current_q_[i] - start_q_[i]));
        }

        const auto now_steady =
            std::chrono::steady_clock::now();

        if (!moving_) {
            if (max_joint_difference < 0.00015) {
                return;
            }

            moving_ = true;
            motion_start_time_ = now_steady;

            RCLCPP_INFO(
                get_logger(), "Robot motion detected");
        }

        KDL::Frame actual;

        if (!compute_fk(current_q_, actual)) {
            finish(false, "FK computation failed");
            return;
        }

        ++sample_count_;

        const KDL::Vector path =
            goal_pose_.p - start_pose_.p;

        const KDL::Vector relative =
            actual.p - start_pose_.p;

        const double denominator =
            path.x() * path.x() +
            path.y() * path.y() +
            path.z() * path.z();

        const double projection =
            (relative.x() * path.x() +
             relative.y() * path.y() +
             relative.z() * path.z()) / denominator;

        const double s =
            std::clamp(projection, 0.0, 1.0);

        const KDL::Vector nearest =
            start_pose_.p + path * s;

        // 实测位置到规划直线段的最短距离
        const double cross_track =
            (actual.p - nearest).Norm();

        // 末端姿态相对起点的偏差
        const double orientation_error =
            KDL::diff(start_pose_, actual).rot.Norm();

        const double endpoint_error =
            (actual.p - goal_pose_.p).Norm();

        max_cross_track_ =
            std::max(max_cross_track_, cross_track);

        max_orientation_error_ =
            std::max(max_orientation_error_, orientation_error);

        max_progress_ =
            std::max(max_progress_, projection);

        if (endpoint_error < 0.0003 &&
            orientation_error < 0.003) {

            if (!near_goal_) {
                near_goal_ = true;
                near_goal_since_ = now_steady;
            }

            if (now_steady - near_goal_since_ >= 800ms) {
                final_position_error_ = endpoint_error;
                final_orientation_error_ =
                    KDL::diff(goal_pose_, actual).rot.Norm();

                const bool passed =
                    sample_count_ >= 20 &&
                    max_cross_track_ <= 0.0005 &&
                    max_orientation_error_ <= 0.01 &&
                    final_position_error_ <= 0.0003 &&
                    final_orientation_error_ <= 0.003;

                finish(
                    passed,
                    passed ? "Path checks passed" :
                             "Path error threshold exceeded");

                return;
            }
        } else {
            near_goal_ = false;
        }
    }

    void tick()
    {
        if (done_) {
            return;
        }

        if (!armed_ && pending_preview_ &&
            model_ready_ && state_ready_) {

            if (std::chrono::steady_clock::now() -
                last_state_time_ > 1s) {
                return;
            }

            const auto &preview = *pending_preview_;

            for (size_t i = 0; i < 6; ++i) {
                start_q_[i] =
                    preview.points.front().positions[i];

                goal_q_[i] =
                    preview.points.back().positions[i];

                if (std::abs(current_q_[i] - start_q_[i]) >
                    0.005) {
                    finish(false, "Trajectory start mismatch");
                    return;
                }
            }

            if (!compute_fk(start_q_, start_pose_) ||
                !compute_fk(goal_q_, goal_pose_)) {
                finish(false, "Preview FK failed");
                return;
            }

            const double length =
                (goal_pose_.p - start_pose_.p).Norm();

            const double rotation_difference =
                KDL::diff(start_pose_, goal_pose_).rot.Norm();

            if (length < 0.002 ||
                length > 0.030 ||
                rotation_difference > 0.005) {
                finish(false, "Unexpected preview path");
                return;
            }

            armed_ = true;
            armed_time_ = std::chrono::steady_clock::now();

            RCLCPP_INFO(
                get_logger(),
                "Monitor armed: path length %.3f mm",
                length * 1000.0);

            RCLCPP_INFO(
                get_logger(),
                "Execute the Cartesian trajectory now");
        }

        if (!armed_) {
            return;
        }

        const auto time_now =
            std::chrono::steady_clock::now();

        if (!moving_ && time_now - armed_time_ > 30s) {
            finish(false, "Motion not detected within 30 seconds");
        } else if (moving_ &&
                   time_now - motion_start_time_ > 20s) {
            finish(false, "Motion monitoring timeout");
        }
    }

    void finish(bool passed, const std::string &reason)
    {
        if (done_) {
            return;
        }

        done_ = true;
        passed_ = passed;

        RCLCPP_INFO(
            get_logger(), "========================================");

        RCLCPP_INFO(
            get_logger(),
            "Captured samples: %zu", sample_count_);

        RCLCPP_INFO(
            get_logger(),
            "Maximum cross-track error: %.6f mm",
            max_cross_track_ * 1000.0);

        RCLCPP_INFO(
            get_logger(),
            "Maximum orientation error: %.6f rad",
            max_orientation_error_);

        RCLCPP_INFO(
            get_logger(),
            "Maximum path progress: %.3f%%",
            max_progress_ * 100.0);

        RCLCPP_INFO(
            get_logger(),
            "Final position error: %.6f mm",
            final_position_error_ * 1000.0);

        RCLCPP_INFO(
            get_logger(),
            "Final orientation error: %.6f rad",
            final_orientation_error_);

        if (passed) {
            RCLCPP_INFO(
                get_logger(),
                "SUCCESS: %s", reason.c_str());
        } else {
            RCLCPP_ERROR(
                get_logger(),
                "FAILED: %s", reason.c_str());
        }

        RCLCPP_INFO(
            get_logger(), "========================================");

        rclcpp::shutdown();
    }

    KDL::Chain chain_;

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;

    KDL::Frame start_pose_;
    KDL::Frame goal_pose_;

    std::array<double, 6> current_q_{};
    std::array<double, 6> start_q_{};
    std::array<double, 6> goal_q_{};

    int64_t monitor_start_ns_{0};

    bool model_ready_{false};
    bool state_ready_{false};
    bool armed_{false};
    bool moving_{false};
    bool near_goal_{false};
    bool done_{false};
    bool passed_{false};

    size_t sample_count_{0};

    double max_cross_track_{0.0};
    double max_orientation_error_{0.0};
    double max_progress_{0.0};

    double final_position_error_{0.0};
    double final_orientation_error_{0.0};

    std::chrono::steady_clock::time_point last_state_time_{};
    std::chrono::steady_clock::time_point armed_time_{};
    std::chrono::steady_clock::time_point motion_start_time_{};
    std::chrono::steady_clock::time_point near_goal_since_{};

    Trajectory::SharedPtr pending_preview_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        urdf_sub_;

    rclcpp::Subscription<JointState>::SharedPtr
        joint_sub_;

    rclcpp::Subscription<Trajectory>::SharedPtr
        preview_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CartesianMonitor>();

    rclcpp::spin(node);

    const bool passed = node->passed();

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    return passed ? 0 : 1;
}