#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <tf2/time.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <urdf/model.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// ============================================================
// MTC 5B - AUBO S3 TCP-space Cartesian planner (ROS 2 Jazzy)
//
// Plan a fixed-orientation straight line at the V1 tongue TCP.
//   * Input displacement: tool_tcp-local X/Y/Z, in metres.
//   * KDL IK target: base_link -> ee_link.
//   * Tool offset: read via TF ee_link -> tool_tcp.
//   * Output: /mtc/tcp_cartesian_preview ONLY.
//
// IMPORTANT:
//   - Preview-only. No action clients or controller commands.
//   - Does not publish to the historical /mtc/cartesian_preview.
//   - 5B is limited to the previously checked NOMINAL transform.
//   - No self/environment collision or acceleration checks.
//   - Do not use for physical robot contact or battery handling.
// ============================================================

class CartesianTrajectoryNode final : public rclcpp::Node
{
public:
  CartesianTrajectoryNode()
  : Node("mtc_tcp_cartesian_trajectory"),
    current_q_(6), q_min_(6), q_max_(6)
  {
    dx_ = declare_parameter<double>("dx", 0.010);
    dy_ = declare_parameter<double>("dy", 0.0);
    dz_ = declare_parameter<double>("dz", 0.0);
    steps_ = declare_parameter<int>("steps", 20);
    duration_ = declare_parameter<double>("duration", 5.0);
    max_joint_step_ = declare_parameter<double>("max_joint_step", 0.20);

    // Explicit opt-in: transform is NOT physically calibrated.
    confirm_nominal_tool_tf_ =
      declare_parameter<bool>("confirm_nominal_tool_tf", false);
    tf_wait_seconds_ = declare_parameter<double>("tf_wait_seconds", 10.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) { on_urdf(msg); });

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        on_joint_state(msg);
      });

    preview_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/mtc/tcp_cartesian_preview", rclcpp::QoS(1).reliable().transient_local());

    start_wait_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(200ms, [this]() { run_planner(); });

    RCLCPP_INFO(get_logger(), "MTC 5B: V1 TCP Cartesian planner (PREVIEW ONLY)");
    RCLCPP_INFO(get_logger(),
      "TCP-local displacement: [%.3f, %.3f, %.3f] mm, steps=%d, time=%.2fs",
      dx_ * 1e3, dy_ * 1e3, dz_ * 1e3, steps_, duration_);
    RCLCPP_WARN(get_logger(),
      "Nominal ee_link->tool_tcp ONLY; uncalibrated, collision checks missing");
    RCLCPP_WARN(get_logger(),
      "Outputs ONLY /mtc/tcp_cartesian_preview; no robot movement command");
  }

  bool passed() const { return success_; }

private:
  void fail(const std::string &reason)
  {
    RCLCPP_ERROR(get_logger(), "TCP PLANNING FAILED: %s", reason.c_str());
    planned_ = true;
    success_ = false;
    rclcpp::shutdown();
  }

  void on_urdf(const std_msgs::msg::String::SharedPtr msg)
  {
    if (model_ready_) return;

    urdf::Model robot;
    if (!robot.initString(msg->data)) {
      fail("URDF parsing failed");
      return;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(msg->data, tree) ||
        !tree.getChain("base_link", "ee_link", chain_)) {
      fail("Cannot construct base_link -> ee_link KDL chain");
      return;
    }
    if (chain_.getNrOfJoints() != 6) {
      fail("Expected exactly six movable KDL joints");
      return;
    }

    const std::array<std::string, 6> expected = {
      "shoulder_joint", "upperArm_joint", "foreArm_joint",
      "wrist1_joint", "wrist2_joint", "wrist3_joint"
    };

    unsigned int index = 0;
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
      const auto &joint = chain_.getSegment(i).getJoint();
      if (joint.getType() == KDL::Joint::None) continue;
      const std::string name = joint.getName();
      if (index >= expected.size() || name != expected[index]) {
        fail("Unexpected KDL joint order at: " + name);
        return;
      }

      const auto urdf_joint = robot.getJoint(name);
      if (!urdf_joint || urdf_joint->type != urdf::Joint::REVOLUTE ||
          !urdf_joint->limits) {
        fail("Missing revolute joint limits: " + name);
        return;
      }
      const auto &lim = *urdf_joint->limits;
      if (!std::isfinite(lim.lower) || !std::isfinite(lim.upper) ||
          !std::isfinite(lim.velocity) ||
          lim.lower >= lim.upper || lim.velocity <= 0.0) {
        fail("Invalid URDF limits: " + name);
        return;
      }
      q_min_(index) = lim.lower;
      q_max_(index) = lim.upper;
      velocity_limits_.push_back(lim.velocity);
      joint_names_.push_back(name);
      ++index;
    }

    if (index != 6) {
      fail("KDL joint extraction incomplete");
      return;
    }

    model_ready_ = true;
    RCLCPP_INFO(get_logger(), "KDL ready: %u segments, six joints",
      chain_.getNrOfSegments());
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!model_ready_ || planned_) return;

    KDL::JntArray next_q(6);
    for (std::size_t i = 0; i < 6; ++i) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) return;
      const std::size_t pos = std::distance(msg->name.begin(), it);
      if (pos >= msg->position.size()) return;
      const double value = msg->position[pos];
      if (!std::isfinite(value) || value < q_min_(i) || value > q_max_(i)) return;
      next_q(i) = value;
    }
    current_q_ = next_q;
    state_received_ = true;
    last_state_time_ = std::chrono::steady_clock::now();
  }

  bool read_nominal_ee_to_tcp(KDL::Frame &ee_tcp)
  {
    geometry_msgs::msg::TransformStamped msg;
    try {
      msg = tf_buffer_->lookupTransform(
        "ee_link", "tool_tcp", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      const double wait = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_wait_).count();
      if (wait >= tf_wait_seconds_) {
        fail(std::string("TF ee_link->tool_tcp unavailable: ") + ex.what());
      } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting for TF ee_link -> tool_tcp ...");
      }
      return false;
    }

    const auto &p = msg.transform.translation;
    const auto &q = msg.transform.rotation;
    const double q_norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q_norm) || q_norm < 1e-9) {
      fail("Non-finite / invalid TCP transform");
      return false;
    }

    const KDL::Vector t(p.x, p.y, p.z);
    const KDL::Rotation R = KDL::Rotation::Quaternion(
      q.x/q_norm, q.y/q_norm, q.z/q_norm, q.w/q_norm);
    ee_tcp = KDL::Frame(R, t);

    // Strict 5B safety gate: test ONLY the nominally verified transform.
    // Nominal: xyz=(0.036437755, 0, 0.115) metres; 180 deg about EE X.
    // This DOES NOT assert that the real tool has been calibrated.
    constexpr double kPi = 3.14159265358979323846;
    const KDL::Frame nominal(
      KDL::Rotation::RotX(kPi), KDL::Vector(0.036437755, 0.0, 0.115));
    const KDL::Twist delta = KDL::diff(nominal, ee_tcp);
    const double pos_delta_m = delta.vel.Norm();
    const double rot_delta_rad = delta.rot.Norm();
    if (!std::isfinite(pos_delta_m) || !std::isfinite(rot_delta_rad) ||
        pos_delta_m > 0.001 || rot_delta_rad > 0.01) {
      fail("ee_link->tool_tcp does not match the verified NOMINAL V1 TF; "
           "stop and inspect frame publishers/parameters");
      return false;
    }
    return true;
  }

  bool validate_parameters()
  {
    if (!confirm_nominal_tool_tf_) {
      fail("Explicit acknowledgement required: "
           "-p confirm_nominal_tool_tf:=true (PREVIEW ONLY)");
      return false;
    }
    if (!std::isfinite(dx_) || !std::isfinite(dy_) ||
        !std::isfinite(dz_) || !std::isfinite(duration_) ||
        !std::isfinite(max_joint_step_) || !std::isfinite(tf_wait_seconds_)) {
      fail("Non-finite planning parameter");
      return false;
    }
    const double distance = std::sqrt(dx_*dx_ + dy_*dy_ + dz_*dz_);
    if (distance < 1e-6 || distance > 0.030) {
      fail("TCP displacement must be > 0 and <= 30 mm");
      return false;
    }
    if (steps_ < 5 || steps_ > 200 || duration_ < 1.0 || duration_ > 30.0 ||
        max_joint_step_ <= 0.0 || max_joint_step_ > 0.5 ||
        tf_wait_seconds_ < 1.0 || tf_wait_seconds_ > 120.0) {
      fail("Invalid steps, duration, max_joint_step or tf_wait_seconds");
      return false;
    }
    return true;
  }

  void run_planner()
  {
    if (planned_) return;
    if (!validate_parameters()) return;
    if (!model_ready_ || !state_received_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
        "Waiting for URDF and fresh /joint_states ...");
      return;
    }
    if (std::chrono::steady_clock::now() - last_state_time_ > 1s) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Joint state stale; waiting for fresh data");
      return;
    }

    KDL::Frame ee_tcp;
    if (!read_nominal_ee_to_tcp(ee_tcp)) return;
    if (planned_) return;

    KDL::ChainFkSolverPos_recursive fk_solver(chain_);
    KDL::ChainIkSolverVel_pinv ik_vel(chain_);
    KDL::ChainIkSolverPos_NR_JL ik_solver(
      chain_, q_min_, q_max_, fk_solver, ik_vel, 300, 1e-6);

    KDL::Frame start_ee;
    if (fk_solver.JntToCart(current_q_, start_ee) < 0) {
      fail("Start EE FK failed");
      return;
    }
    const KDL::Frame start_tcp = start_ee * ee_tcp;
    const KDL::Vector tcp_local_displacement(dx_, dy_, dz_);
    const KDL::Vector base_displacement = start_tcp.M * tcp_local_displacement;
    const double distance = tcp_local_displacement.Norm();

    RCLCPP_INFO(get_logger(), "-------- TCP Cartesian planning --------");
    RCLCPP_INFO(get_logger(), "Start TCP in base: [%.6f, %.6f, %.6f] m",
      start_tcp.p.x(), start_tcp.p.y(), start_tcp.p.z());
    RCLCPP_INFO(get_logger(), "Requested TCP-local delta: [%.6f, %.6f, %.6f] m",
      dx_, dy_, dz_);
    RCLCPP_INFO(get_logger(), "Equivalent BASE delta: [%.6f, %.6f, %.6f] m",
      base_displacement.x(), base_displacement.y(), base_displacement.z());
    RCLCPP_INFO(get_logger(), "Goal TCP in base: [%.6f, %.6f, %.6f] m",
      start_tcp.p.x() + base_displacement.x(),
      start_tcp.p.y() + base_displacement.y(),
      start_tcp.p.z() + base_displacement.z());

    // Never publish to /mtc/cartesian_preview: the old executor subscribes there.
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.header.frame_id = "base_link";
    trajectory.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint p0;
    p0.positions.resize(6);
    for (std::size_t j = 0; j < 6; ++j) p0.positions[j] = current_q_(j);
    p0.time_from_start = rclcpp::Duration::from_seconds(0.0);
    trajectory.points.push_back(p0);

    KDL::JntArray previous_q = current_q_;
    const double dt = duration_ / static_cast<double>(steps_);
    double max_fk_pos_mm = 0.0;
    double max_fk_rot_rad = 0.0;
    double max_midpoint_mm = 0.0;
    double max_midpoint_rot_rad = 0.0;
    double max_joint_delta = 0.0;
    double max_vel_ratio = 0.0;

    for (int i = 1; i <= steps_; ++i) {
      const double s = static_cast<double>(i) / static_cast<double>(steps_);
      // Straight TCP segment in base_link; hold TCP orientation constant.
      KDL::Frame desired_tcp = start_tcp;
      desired_tcp.p = start_tcp.p + base_displacement * s;
      // Convert TCP target into KDL's base_link -> ee_link IK target.
      const KDL::Frame desired_ee = desired_tcp * ee_tcp.Inverse();

      KDL::JntArray solved_q(6);
      const int ik_ret = ik_solver.CartToJnt(previous_q, desired_ee, solved_q);
      if (ik_ret < 0) {
        fail("IK failed at TCP waypoint " + std::to_string(i) +
             ", code=" + std::to_string(ik_ret) +
             "; check whether the current mock posture is singular");
        return;
      }

      for (std::size_t j = 0; j < 6; ++j) {
        const double q = solved_q(j);
        if (!std::isfinite(q) || q < q_min_(j) || q > q_max_(j)) {
          fail("Joint limit violated at waypoint " + std::to_string(i));
          return;
        }
        const double dq = std::abs(q - previous_q(j));
        max_joint_delta = std::max(max_joint_delta, dq);
        if (dq > max_joint_step_) {
          fail("Joint discontinuity: waypoint " + std::to_string(i) +
               ", joint " + joint_names_[j]);
          return;
        }
        const double ratio = dq / dt / velocity_limits_[j];
        max_vel_ratio = std::max(max_vel_ratio, ratio);
        if (ratio > 0.5) {
          fail("Estimated joint velocity >50% URDF limit: " + joint_names_[j]);
          return;
        }
      }

      KDL::Frame actual_ee;
      if (fk_solver.JntToCart(solved_q, actual_ee) < 0) {
        fail("Waypoint EE FK failed");
        return;
      }
      const KDL::Frame actual_tcp = actual_ee * ee_tcp;
      const KDL::Twist error = KDL::diff(desired_tcp, actual_tcp);
      const double pos_mm = error.vel.Norm() * 1e3;
      const double rot_rad = error.rot.Norm();
      max_fk_pos_mm = std::max(max_fk_pos_mm, pos_mm);
      max_fk_rot_rad = std::max(max_fk_rot_rad, rot_rad);
      if (!std::isfinite(pos_mm) || !std::isfinite(rot_rad) ||
          pos_mm > 0.5 || rot_rad > 0.002) {
        fail("TCP FK error exceeds limit at waypoint " + std::to_string(i));
        return;
      }

      // Test linear interpolation midpoint in JOINT space at the TOOL TCP.
      KDL::JntArray midpoint_q(6);
      for (std::size_t j = 0; j < 6; ++j)
        midpoint_q(j) = 0.5 * (previous_q(j) + solved_q(j));
      KDL::Frame midpoint_ee;
      if (fk_solver.JntToCart(midpoint_q, midpoint_ee) < 0) {
        fail("Midpoint EE FK failed");
        return;
      }
      const KDL::Frame midpoint_actual_tcp = midpoint_ee * ee_tcp;
      const double mid_s = (static_cast<double>(i) - 0.5) /
        static_cast<double>(steps_);
      KDL::Frame midpoint_desired_tcp = start_tcp;
      midpoint_desired_tcp.p = start_tcp.p + base_displacement * mid_s;
      const KDL::Twist mid_error =
        KDL::diff(midpoint_desired_tcp, midpoint_actual_tcp);
      const double mid_mm = mid_error.vel.Norm() * 1e3;
      const double mid_rot = mid_error.rot.Norm();
      max_midpoint_mm = std::max(max_midpoint_mm, mid_mm);
      max_midpoint_rot_rad = std::max(max_midpoint_rot_rad, mid_rot);
      if (!std::isfinite(mid_mm) || !std::isfinite(mid_rot) ||
          mid_mm > 0.5 || mid_rot > 0.002) {
        fail("TCP joint-interpolation midpoint deviates at waypoint " +
             std::to_string(i));
        return;
      }

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.resize(6);
      for (std::size_t j = 0; j < 6; ++j)
        point.positions[j] = solved_q(j);
      point.time_from_start = rclcpp::Duration::from_seconds(dt * i);
      trajectory.points.push_back(point);

      if (i == 1 || i % 5 == 0 || i == steps_) {
        RCLCPP_INFO(get_logger(),
          "TCP waypoint %02d/%02d: BASE xyz=[%.4f, %.4f, %.4f] m, err=%.5f mm",
          i, steps_, actual_tcp.p.x(), actual_tcp.p.y(), actual_tcp.p.z(), pos_mm);
      }
      previous_q = solved_q;
    }

    // Publish only after every validation has passed.
    preview_pub_->publish(trajectory);
    planned_ = true;
    success_ = true;

    RCLCPP_INFO(get_logger(), "========== TCP PLANNING SUCCESS ==========");
    RCLCPP_INFO(get_logger(), "TCP distance: %.3f mm; points: %zu; time: %.2f s",
      distance * 1e3, trajectory.points.size(), duration_);
    RCLCPP_INFO(get_logger(), "Max TCP FK error: %.6f mm, %.6f rad",
      max_fk_pos_mm, max_fk_rot_rad);
    RCLCPP_INFO(get_logger(), "Max TCP midpoint error: %.6f mm, %.6f rad",
      max_midpoint_mm, max_midpoint_rot_rad);
    RCLCPP_INFO(get_logger(), "Max joint step: %.6f rad; vel ratio: %.2f%%",
      max_joint_delta, max_vel_ratio * 100.0);
    RCLCPP_INFO(get_logger(), "Published: /mtc/tcp_cartesian_preview");
    RCLCPP_WARN(get_logger(),
      "PREVIEW ONLY. Nominal TF; no collision/acceleration/dynamics checks.");
    RCLCPP_WARN(get_logger(), "No controller command, no robot motion.");
    RCLCPP_INFO(get_logger(), "Node alive for topic inspection; Ctrl+C to exit.");
  }

  KDL::Chain chain_;
  KDL::JntArray current_q_, q_min_, q_max_;
  std::vector<std::string> joint_names_;
  std::vector<double> velocity_limits_;

  double dx_{0.010}, dy_{0.0}, dz_{0.0};
  int steps_{20};
  double duration_{5.0};
  double max_joint_step_{0.20};
  double tf_wait_seconds_{10.0};
  bool confirm_nominal_tool_tf_{false};
  bool model_ready_{false};
  bool state_received_{false};
  bool planned_{false};
  bool success_{false};

  std::chrono::steady_clock::time_point last_state_time_{};
  std::chrono::steady_clock::time_point start_wait_{};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr preview_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianTrajectoryNode>();
  rclcpp::spin(node);
  const bool passed = node->passed();
  if (rclcpp::ok()) rclcpp::shutdown();
  return passed ? 0 : 1;
}
