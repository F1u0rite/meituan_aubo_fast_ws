#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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

// MTC 5C read-only TCP execution monitor. It independently checks a new
// planner preview using KDL FK + the nominal 5A TF, publishes a stamp-matched
// 'armed' acknowledgment, then checks actual /joint_states at the TOOL TCP.
// No hardware commands. All checks in Mock are numerical, NOT physical safety.
namespace {
const std::array<std::string, 6> kJoints = {
  "shoulder_joint", "upperArm_joint", "foreArm_joint",
  "wrist1_joint", "wrist2_joint", "wrist3_joint"
};

double point_seconds(const trajectory_msgs::msg::JointTrajectoryPoint &p)
{
  return static_cast<double>(p.time_from_start.sec) +
         static_cast<double>(p.time_from_start.nanosec) * 1e-9;
}
}

class TcpCartesianMonitor final : public rclcpp::Node
{
public:
  TcpCartesianMonitor() : Node("mtc_tcp_cartesian_monitor")
  {
    started_ = now();
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    armed_pub_ = create_publisher<std_msgs::msg::Header>(
      "/mtc/tcp_monitor_armed", rclcpp::QoS(1).reliable().transient_local());
    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::SharedPtr msg) { on_urdf(msg); });
    joint_sub_ = create_subscription<JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](JointState::SharedPtr msg) { on_state(msg); });
    preview_sub_ = create_subscription<Trajectory>(
      "/mtc/tcp_cartesian_preview", rclcpp::QoS(1).reliable().transient_local(),
      [this](Trajectory::SharedPtr msg) { on_preview(msg); });
    timer_ = create_wall_timer(100ms, [this]() { tick(); });
    RCLCPP_INFO(get_logger(),
      "5C TCP monitor READ ONLY. Start planner AFTER monitor/executor start.");
  }

  bool passed() const { return passed_; }

private:
  void fail(const std::string &reason)
  {
    finish(false, reason);
  }

  void on_urdf(const std_msgs::msg::String::SharedPtr msg)
  {
    if (model_ready_ || done_) return;
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(msg->data, tree) ||
        !tree.getChain("base_link", "ee_link", chain_) ||
        chain_.getNrOfJoints() != 6) {
      fail("Invalid base_link -> ee_link KDL chain");
      return;
    }
    size_t j = 0;
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
      const auto &joint = chain_.getSegment(i).getJoint();
      if (joint.getType() == KDL::Joint::None) continue;
      if (j >= 6 || joint.getName() != kJoints[j]) {
        fail("Unexpected KDL joint names/order");
        return;
      }
      ++j;
    }
    if (j != 6) {
      fail("KDL joint count mismatch");
      return;
    }
    fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    model_ready_ = true;
    RCLCPP_INFO(get_logger(), "KDL FK chain ready");
  }

  bool fk_tcp(const std::array<double, 6> &positions, KDL::Frame &pose)
  {
    if (!fk_) return false;
    KDL::JntArray joints(6);
    for (size_t j = 0; j < 6; ++j) joints(j) = positions[j];
    KDL::Frame ee;
    if (fk_->JntToCart(joints, ee) < 0) return false;
    pose = ee * ee_tcp_;
    return true;
  }

  bool read_nominal_tool_tf()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = buffer_->lookupTransform("ee_link", "tool_tcp", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for ee_link -> tool_tcp: %s", ex.what());
      return false;
    }
    const auto &t = tf.transform.translation;
    const auto &q = tf.transform.rotation;
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
        !std::isfinite(norm) || norm < 1e-9) {
      fail("Invalid tool TF");
      return false;
    }
    const KDL::Rotation R = KDL::Rotation::Quaternion(
      q.x/norm, q.y/norm, q.z/norm, q.w/norm);
    ee_tcp_ = KDL::Frame(R, KDL::Vector(t.x, t.y, t.z));
    constexpr double pi = 3.14159265358979323846;
    const KDL::Frame nominal(
      KDL::Rotation::RotX(pi), KDL::Vector(0.036437755, 0., 0.115));
    const KDL::Twist d = KDL::diff(nominal, ee_tcp_);
    if (!std::isfinite(d.vel.Norm()) || !std::isfinite(d.rot.Norm()) ||
        d.vel.Norm() > 0.001 || d.rot.Norm() > 0.01) {
      fail("Tool TF differs from NOMINAL V1 calibration (cannot arm)");
      return false;
    }
    return true;
  }

  void on_preview(const Trajectory::SharedPtr msg)
  {
    if (armed_ || done_ || pending_) return;
    if (msg->header.stamp.sec <= 0 ||
        msg->header.frame_id != "base_link" ||
        (rclcpp::Time(msg->header.stamp) - started_).seconds() < -0.10 ||
        (now() - rclcpp::Time(msg->header.stamp)).seconds() > 15.0 ||
        (now() - rclcpp::Time(msg->header.stamp)).seconds() < -1.0) {
      RCLCPP_WARN(get_logger(), "Ignoring stale/future TCP preview");
      return;
    }
    if (msg->joint_names.size() != 6 ||
        !std::equal(kJoints.begin(), kJoints.end(), msg->joint_names.begin()) ||
        msg->points.size() < 6 || msg->points.size() > 201 ||
        std::abs(point_seconds(msg->points.front())) > 1e-9) {
      fail("Invalid TCP preview structure");
      return;
    }
    const double duration = point_seconds(msg->points.back());
    if (duration < 2.0 || duration > 10.0 || !std::isfinite(duration)) {
      fail("Unexpected preview duration");
      return;
    }
    for (size_t i = 0; i < msg->points.size(); ++i) {
      const auto &p = msg->points[i];
      if (p.positions.size() != 6 || !p.velocities.empty() ||
          !p.accelerations.empty() || !p.effort.empty() ||
          (i && point_seconds(p) <= point_seconds(msg->points[i-1]))) {
        fail("Invalid TCP preview point/time");
        return;
      }
      for (double q : p.positions) {
        if (!std::isfinite(q)) {
          fail("Non-finite TCP preview position");
          return;
        }
      }
    }
    pending_ = msg;
    preview_received_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "NEW TCP preview received, validating FK path");
  }

  void on_state(const JointState::SharedPtr msg)
  {
    if (done_) return;
    std::array<double, 6> q{};
    for (size_t j = 0; j < 6; ++j) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), kJoints[j]);
      if (it == msg->name.end()) return;
      const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
      if (idx >= msg->position.size() || !std::isfinite(msg->position[idx]))
        return;
      q[j] = msg->position[idx];
    }
    current_ = q;
    state_ready_ = true;
    last_state_ = std::chrono::steady_clock::now();
    if (!armed_) return;

    double joint_departure = 0.0;
    for (size_t j = 0; j < 6; ++j)
      joint_departure = std::max(joint_departure,
        std::abs(current_[j] - start_q_[j]));
    const auto steady = std::chrono::steady_clock::now();
    if (!moving_) {
      if (joint_departure < 0.00015) return;
      moving_ = true;
      moved_at_ = steady;
      RCLCPP_INFO(get_logger(), "TCP motion detected");
    }
    KDL::Frame actual;
    if (!fk_tcp(current_, actual)) {
      fail("Live TCP FK failed");
      return;
    }
    ++samples_;
    const KDL::Vector segment = goal_pose_.p - start_pose_.p;
    const double denom = KDL::dot(segment, segment);
    if (denom < 1e-12) {
      fail("Zero-length monitored path");
      return;
    }
    const double projected = KDL::dot(actual.p - start_pose_.p, segment) / denom;
    const double s = std::clamp(projected, 0.0, 1.0);
    const KDL::Vector closest = start_pose_.p + segment * s;
    const double cross_track = (actual.p - closest).Norm();
    const double orient = KDL::diff(start_pose_, actual).rot.Norm();
    const double remaining = (goal_pose_.p - actual.p).Norm();
    if (!std::isfinite(cross_track) || !std::isfinite(orient) ||
        !std::isfinite(remaining)) {
      fail("Non-finite TCP tracking error");
      return;
    }
    max_cross_ = std::max(max_cross_, cross_track);
    max_orient_ = std::max(max_orient_, orient);
    max_progress_ = std::max(max_progress_, projected);
    if (cross_track > 0.0005 || orient > 0.01) {
      fail("TCP tracking threshold exceeded");
      return;
    }
    if (remaining < 0.0003 && orient < 0.003) {
      if (!near_goal_) {
        near_goal_ = true;
        near_since_ = steady;
      }
      if (steady - near_since_ >= 600ms) {
        end_error_ = remaining;
        const bool pass = samples_ >= 20 && max_progress_ >= 0.98 &&
          max_cross_ <= 0.0005 && max_orient_ <= 0.01;
        finish(pass, pass ? "Mock TCP path checks passed" :
          "Insufficient motion evidence / endpoint verification");
      }
    } else {
      near_goal_ = false;
    }
  }

  void tick()
  {
    if (done_) return;
    const auto steady = std::chrono::steady_clock::now();
    if (!armed_ && pending_ && model_ready_ && state_ready_) {
      if (steady - last_state_ > 500ms) return;
      if ((now() - rclcpp::Time(pending_->header.stamp)).seconds() > 15.0) {
        fail("TCP preview expired before arming");
        return;
      }
      if (!read_nominal_tool_tf()) {
        if (!done_ && steady - preview_received_ > 10s)
          fail("Tool TF unavailable 10 seconds after preview");
        return;
      }
      const auto &points = pending_->points;
      for (size_t j = 0; j < 6; ++j) {
        start_q_[j] = points.front().positions[j];
        goal_q_[j] = points.back().positions[j];
        if (std::abs(start_q_[j] - current_[j]) > 0.005) {
          fail("Joint state differs from TCP preview start");
          return;
        }
      }
      if (!fk_tcp(start_q_, start_pose_) || !fk_tcp(goal_q_, goal_pose_)) {
        fail("TCP endpoint FK failure");
        return;
      }
      const double length = (goal_pose_.p - start_pose_.p).Norm();
      const double rot_delta = KDL::diff(start_pose_, goal_pose_).rot.Norm();
      if (!std::isfinite(length) || length < 0.002 || length > 0.030 ||
          rot_delta > 0.005) {
        fail("TCP preview length/rotation invalid");
        return;
      }
      // Verify that all PREVIEW TCP waypoints lie on the same segment and
      // maintain orientation; reject unrelated joint trajectories on this topic.
      for (size_t i = 0; i < points.size(); ++i) {
        std::array<double, 6> q{};
        for (size_t j = 0; j < 6; ++j) q[j] = points[i].positions[j];
        KDL::Frame pose;
        if (!fk_tcp(q, pose)) {
          fail("TCP preview waypoint FK failure");
          return;
        }
        const double s = static_cast<double>(i) / (points.size() - 1);
        const KDL::Vector expected = start_pose_.p +
          (goal_pose_.p - start_pose_.p) * s;
        if ((pose.p - expected).Norm() > 0.0005 ||
            KDL::diff(start_pose_, pose).rot.Norm() > 0.002) {
          fail("TCP preview waypoint deviates from straight line");
          return;
        }
      }
      armed_ = true;
      armed_at_ = steady;
      armed_pub_->publish(pending_->header);
      RCLCPP_INFO(get_logger(),
        "MONITOR ARMED for preview stamp %d.%09u (%.3f mm). "
        "Executor may send goal on matching stamp.",
        pending_->header.stamp.sec, pending_->header.stamp.nanosec,
        length * 1e3);
    }
    if (!armed_) return;
    if (!moving_ && steady - armed_at_ > 20s) {
      fail("No mock motion detected within 20s of monitor arming");
    } else if (moving_ && steady - moved_at_ > 20s) {
      fail("Mock TCP monitoring timed out");
    } else if (steady - last_state_ > 1s) {
      fail("Joint states lost while monitoring");
    }
  }

  void finish(bool pass, const std::string &reason)
  {
    if (done_) return;
    done_ = true;
    passed_ = pass;
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "TCP samples: %zu", samples_);
    RCLCPP_INFO(get_logger(), "Max TCP cross-track error: %.6f mm", max_cross_*1e3);
    RCLCPP_INFO(get_logger(), "Max TCP orientation error: %.6f rad", max_orient_);
    RCLCPP_INFO(get_logger(), "Max progress: %.3f%%", max_progress_*100.0);
    RCLCPP_INFO(get_logger(), "Final TCP error: %.6f mm", end_error_*1e3);
    if (pass) RCLCPP_INFO(get_logger(), "5C TCP MONITOR SUCCESS: %s", reason.c_str());
    else RCLCPP_ERROR(get_logger(), "5C TCP MONITOR FAILED: %s", reason.c_str());
    RCLCPP_INFO(get_logger(), "========================================");
    rclcpp::shutdown();
  }

  rclcpp::Time started_{0, 0, RCL_ROS_TIME};
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  KDL::Frame ee_tcp_, start_pose_, goal_pose_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::array<double, 6> current_{}, start_q_{}, goal_q_{};
  bool model_ready_{false}, state_ready_{false}, armed_{false};
  bool moving_{false}, near_goal_{false}, done_{false}, passed_{false};
  std::size_t samples_{0};
  double max_cross_{0.}, max_orient_{0.}, max_progress_{0.}, end_error_{0.};
  std::chrono::steady_clock::time_point last_state_{};
  std::chrono::steady_clock::time_point preview_received_{};
  std::chrono::steady_clock::time_point armed_at_{};
  std::chrono::steady_clock::time_point moved_at_{};
  std::chrono::steady_clock::time_point near_since_{};
  Trajectory::SharedPtr pending_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr armed_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr preview_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TcpCartesianMonitor>();
  rclcpp::spin(node);
  const bool ok = node->passed();
  if (rclcpp::ok()) rclcpp::shutdown();
  return ok ? 0 : 1;
}
