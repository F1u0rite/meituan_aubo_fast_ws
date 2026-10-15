#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/time.h>
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
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using Trajectory = trajectory_msgs::msg::JointTrajectory;
using JointState = sensor_msgs::msg::JointState;

// ============================================================
// MTC 5D - V1 tongue TOOL TCP multi-stage planner (PREVIEW ONLY)
//
// Stages (DEMONSTRATION, NOT REAL PICK/PLACE):
// 1 approach: TCP +X
// 2 insert:   TCP +X
// 3 lift:     Base +Z
// 4 transfer: Base +X
// 5 lower:    Base -Z
// 6 withdraw: TCP -X
//
// No actual ring/slot target, collision scene, physical calibration,
// acceleration constraints, or payload dynamics. All offsets are
// deliberately small and editable for isolated mock planning.
//
// Publishes ONLY:
//   /mtc/sequence_preview    (JointTrajectory)
//   /mtc/sequence_tcp_poses  (PoseArray in base_link)
//   /mtc/sequence_stages     (String stage/index/time manifest)
// NO ACTION CLIENT; NO CONTROLLER COMMAND; no topic shared with 5C.
// ============================================================

class TcpSequencePlanner final : public rclcpp::Node
{
public:
  TcpSequencePlanner()
  : Node("mtc_tcp_sequence_planner"), current_q_(6), q_min_(6), q_max_(6)
  {
    confirm_ = declare_parameter<bool>("confirm_nominal_tool_tf", false);
    approach_m_ = declare_parameter<double>("approach_m", 0.005);
    insert_m_ = declare_parameter<double>("insert_m", 0.010);
    lift_m_ = declare_parameter<double>("lift_m", 0.005);
    transfer_x_m_ = declare_parameter<double>("transfer_x_m", 0.005);
    lower_m_ = declare_parameter<double>("lower_m", 0.005);
    withdraw_m_ = declare_parameter<double>("withdraw_m", 0.015);
    steps_per_stage_ = declare_parameter<int>("steps_per_stage", 10);
    seconds_per_stage_ = declare_parameter<double>("seconds_per_stage", 2.0);
    max_joint_step_ = declare_parameter<double>("max_joint_step", 0.10);
    tf_wait_seconds_ = declare_parameter<double>("tf_wait_seconds", 10.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) { on_urdf(msg); });

    joint_sub_ = create_subscription<JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const JointState::SharedPtr msg) { on_joint_state(msg); });

    trajectory_pub_ = create_publisher<Trajectory>(
      "/mtc/sequence_preview", rclcpp::QoS(1).reliable().transient_local());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/mtc/sequence_tcp_poses", rclcpp::QoS(1).reliable().transient_local());
    manifest_pub_ = create_publisher<std_msgs::msg::String>(
      "/mtc/sequence_stages", rclcpp::QoS(1).reliable().transient_local());

    started_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(200ms, [this]() { plan_once(); });

    RCLCPP_WARN(get_logger(),
      "MTC 5D: PREVIEW ONLY. NOMINAL, UNCALIBRATED V1 TCP; NO ROBOT MOTION");
    RCLCPP_WARN(get_logger(),
      "Six mock demonstration stages; NO ring/slot target, collision or dynamics checks");
    RCLCPP_INFO(get_logger(),
      "Stages: approach %.1f, insert %.1f, lift %.1f, transfer %.1f, lower %.1f, "
      "withdraw %.1f mm",
      approach_m_ * 1e3, insert_m_ * 1e3, lift_m_ * 1e3,
      transfer_x_m_ * 1e3, lower_m_ * 1e3, withdraw_m_ * 1e3);
  }

  bool passed() const { return success_; }

private:
  enum class Axes { TCP, BASE };
  struct Stage {
    const char *name;
    Axes axes;
    KDL::Vector delta;
  };

  void fail(const std::string &message)
  {
    if (planned_) return;
    RCLCPP_ERROR(get_logger(), "5D PLANNING FAILED: %s", message.c_str());
    planned_ = true;
    success_ = false;
    rclcpp::shutdown();
  }

  bool parameters_ok()
  {
    if (!confirm_) {
      fail("Require -p confirm_nominal_tool_tf:=true (PREVIEW ONLY)");
      return false;
    }
    const std::array<double, 6> distances = {
      approach_m_, insert_m_, lift_m_, transfer_x_m_, lower_m_, withdraw_m_};
    for (const double d : distances) {
      if (!std::isfinite(d) || d <= 0.0001 || d > 0.020) {
        fail("Every demonstration stage must be >0.1 and <=20 mm");
        return false;
      }
    }
    if (steps_per_stage_ < 5 || steps_per_stage_ > 30 ||
        !std::isfinite(seconds_per_stage_) ||
        seconds_per_stage_ < 1.0 || seconds_per_stage_ > 10.0 ||
        !std::isfinite(max_joint_step_) ||
        max_joint_step_ <= 0.0 || max_joint_step_ > 0.20 ||
        !std::isfinite(tf_wait_seconds_) ||
        tf_wait_seconds_ < 1.0 || tf_wait_seconds_ > 30.0) {
      fail("Invalid steps_per_stage, seconds_per_stage, max_joint_step or TF wait");
      return false;
    }
    return true;
  }

  void on_urdf(const std_msgs::msg::String::SharedPtr msg)
  {
    if (model_ready_ || planned_) return;
    urdf::Model robot;
    KDL::Tree tree;
    if (!robot.initString(msg->data) ||
        !kdl_parser::treeFromString(msg->data, tree) ||
        !tree.getChain("base_link", "ee_link", chain_) ||
        chain_.getNrOfJoints() != 6) {
      fail("Invalid AUBO base_link->ee_link URDF/KDL chain");
      return;
    }
    const std::array<std::string, 6> expected = {
      "shoulder_joint", "upperArm_joint", "foreArm_joint",
      "wrist1_joint", "wrist2_joint", "wrist3_joint"};
    unsigned int index = 0;
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
      const auto &joint = chain_.getSegment(i).getJoint();
      if (joint.getType() == KDL::Joint::None) continue;
      const std::string name = joint.getName();
      if (index >= expected.size() || name != expected[index]) {
        fail("Unexpected joint name or ordering: " + name);
        return;
      }
      const auto urdf_joint = robot.getJoint(name);
      if (!urdf_joint || urdf_joint->type != urdf::Joint::REVOLUTE ||
          !urdf_joint->limits) {
        fail("Missing URDF limits: " + name);
        return;
      }
      const auto &limit = *urdf_joint->limits;
      if (!std::isfinite(limit.lower) || !std::isfinite(limit.upper) ||
          !std::isfinite(limit.velocity) || limit.lower >= limit.upper ||
          limit.velocity <= 0.0) {
        fail("Invalid URDF joint limits: " + name);
        return;
      }
      q_min_(index) = limit.lower;
      q_max_(index) = limit.upper;
      velocity_limits_.push_back(limit.velocity);
      joint_names_.push_back(name);
      ++index;
    }
    if (index != 6) {
      fail("Expected six revolute joints");
      return;
    }
    model_ready_ = true;
    RCLCPP_INFO(get_logger(), "5D KDL ready: %u segments, six joints",
      chain_.getNrOfSegments());
  }

  void on_joint_state(const JointState::SharedPtr msg)
  {
    if (!model_ready_ || planned_) return;
    KDL::JntArray q(6);
    for (std::size_t j = 0; j < 6; ++j) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[j]);
      if (it == msg->name.end()) return;
      const auto index = static_cast<std::size_t>(
        std::distance(msg->name.begin(), it));
      if (index >= msg->position.size()) return;
      const double value = msg->position[index];
      if (!std::isfinite(value) || value < q_min_(j) || value > q_max_(j)) return;
      q(j) = value;
    }
    current_q_ = q;
    state_received_ = true;
    last_state_ = std::chrono::steady_clock::now();
  }

  bool read_nominal_tool(KDL::Frame &ee_tcp)
  {
    geometry_msgs::msg::TransformStamped msg;
    try {
      msg = tf_buffer_->lookupTransform(
        "ee_link", "tool_tcp", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started_).count();
      if (age > tf_wait_seconds_) {
        fail(std::string("ee_link->tool_tcp TF timeout: ") + ex.what());
      } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting for ee_link->tool_tcp TF...");
      }
      return false;
    }
    const auto &t = msg.transform.translation;
    const auto &q = msg.transform.rotation;
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
        !std::isfinite(norm) || norm < 1e-9) {
      fail("Invalid tool TF");
      return false;
    }
    ee_tcp = KDL::Frame(
      KDL::Rotation::Quaternion(q.x/norm, q.y/norm, q.z/norm, q.w/norm),
      KDL::Vector(t.x, t.y, t.z));

    // EXACT SAME *NOMINAL* tool geometry gate as the validated 5B planner.
    constexpr double PI = 3.14159265358979323846;
    const KDL::Frame nominal(
      KDL::Rotation::RotX(PI), KDL::Vector(0.036437755, 0.0, 0.115));
    const KDL::Twist diff = KDL::diff(nominal, ee_tcp);
    if (diff.vel.Norm() > 0.001 || diff.rot.Norm() > 0.01) {
      fail("TCP transform differs from 5B NOMINAL reference; refuse preview");
      return false;
    }
    return true;
  }

  static geometry_msgs::msg::Pose to_pose(const KDL::Frame &frame)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = frame.p.x();
    pose.position.y = frame.p.y();
    pose.position.z = frame.p.z();
    double x = 0., y = 0., z = 0., w = 1.;
    frame.M.GetQuaternion(x, y, z, w);
    pose.orientation.x = x;
    pose.orientation.y = y;
    pose.orientation.z = z;
    pose.orientation.w = w;
    return pose;
  }

  void plan_once()
  {
    if (planned_) return;
    if (!parameters_ok()) return;
    if (!model_ready_ || !state_received_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
        "Waiting for URDF and /joint_states...");
      return;
    }
    if (std::chrono::steady_clock::now() - last_state_ > 1s) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Joint state stale: waiting for recent update");
      return;
    }
    KDL::Frame ee_tcp;
    if (!read_nominal_tool(ee_tcp)) return;
    if (planned_) return;

    KDL::ChainFkSolverPos_recursive fk(chain_);
    KDL::ChainIkSolverVel_pinv ik_vel(chain_);
    KDL::ChainIkSolverPos_NR_JL ik(
      chain_, q_min_, q_max_, fk, ik_vel, 300, 1e-6);

    KDL::Frame ee_start;
    if (fk.JntToCart(current_q_, ee_start) < 0) {
      fail("Start FK failed");
      return;
    }
    KDL::Frame stage_start = ee_start * ee_tcp;
    const KDL::Rotation initial_orientation = stage_start.M;
    KDL::JntArray previous_q = current_q_;

    // Fixed orientation for the demonstration. TCP-local moves are defined
    // using the orientation at the start of each stage (same here).
    const std::array<Stage, 6> stages = {{
      {"01_APPROACH_TCP_X", Axes::TCP, KDL::Vector( approach_m_, 0., 0.)},
      {"02_INSERT_TCP_X", Axes::TCP, KDL::Vector( insert_m_, 0., 0.)},
      {"03_LIFT_BASE_Z", Axes::BASE, KDL::Vector(0., 0., lift_m_)},
      {"04_TRANSFER_BASE_X", Axes::BASE, KDL::Vector(transfer_x_m_, 0., 0.)},
      {"05_LOWER_BASE_Z", Axes::BASE, KDL::Vector(0., 0., -lower_m_)},
      {"06_WITHDRAW_TCP_X", Axes::TCP, KDL::Vector(-withdraw_m_, 0., 0.)}
    }};

    Trajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.header.frame_id = "base_link";
    trajectory.joint_names = joint_names_;

    geometry_msgs::msg::PoseArray poses;
    poses.header = trajectory.header;

    trajectory_msgs::msg::JointTrajectoryPoint first;
    first.positions.resize(6);
    for (std::size_t j = 0; j < 6; ++j) first.positions[j] = current_q_(j);
    first.time_from_start = rclcpp::Duration::from_seconds(0.0);
    trajectory.points.push_back(first);
    poses.poses.push_back(to_pose(stage_start));

    std::ostringstream summary;
    summary << std::fixed << std::setprecision(6);
    summary << "5D PREVIEW ONLY / NOMINAL TCP / NO EXECUTION\n";
    summary << "Stamp=" << trajectory.header.stamp.sec << "."
            << std::setfill('0') << std::setw(9)
            << trajectory.header.stamp.nanosec << std::setfill(' ') << "\n";
    summary << "start index=0 time=0.000000\n";

    const double dt = seconds_per_stage_ / static_cast<double>(steps_per_stage_);
    double elapsed = 0.0;
    double max_fk_mm = 0.0;
    double max_fk_rad = 0.0;
    double max_mid_mm = 0.0;
    double max_mid_rad = 0.0;
    double max_step_rad = 0.0;
    double max_velocity_ratio = 0.0;

    for (std::size_t k = 0; k < stages.size(); ++k) {
      const Stage &stage = stages[k];
      KDL::Vector delta_base = stage.delta;
      if (stage.axes == Axes::TCP) {
        delta_base = stage_start.M * stage.delta;
      }
      const std::size_t begin_index = trajectory.points.size() - 1;
      const KDL::Frame stage_goal(initial_orientation, stage_start.p + delta_base);
      RCLCPP_INFO(get_logger(),
        "5D STAGE %zu/6 %s: BASE delta [%.3f, %.3f, %.3f] mm",
        k + 1, stage.name, delta_base.x() * 1e3, delta_base.y() * 1e3,
        delta_base.z() * 1e3);

      for (int i = 1; i <= steps_per_stage_; ++i) {
        const double s = static_cast<double>(i) /
          static_cast<double>(steps_per_stage_);
        const KDL::Frame wanted_tcp(initial_orientation,
          stage_start.p + delta_base * s);
        const KDL::Frame wanted_ee = wanted_tcp * ee_tcp.Inverse();
        KDL::JntArray solved(6);
        const int ret = ik.CartToJnt(previous_q, wanted_ee, solved);
        if (ret < 0) {
          fail(std::string(stage.name) + " IK failure step " +
               std::to_string(i) + " code=" + std::to_string(ret));
          return;
        }
        for (std::size_t j = 0; j < 6; ++j) {
          const double q = solved(j);
          const double dq = std::abs(q - previous_q(j));
          if (!std::isfinite(q) || q < q_min_(j) || q > q_max_(j)) {
            fail(std::string(stage.name) + " joint limit: " + joint_names_[j]);
            return;
          }
          if (dq > max_joint_step_) {
            fail(std::string(stage.name) + " joint discontinuity: " + joint_names_[j]);
            return;
          }
          const double ratio = dq / dt / velocity_limits_[j];
          if (!std::isfinite(ratio) || ratio > 0.5) {
            fail(std::string(stage.name) + " joint velocity >50% URDF: " +
                 joint_names_[j]);
            return;
          }
          max_step_rad = std::max(max_step_rad, dq);
          max_velocity_ratio = std::max(max_velocity_ratio, ratio);
        }

        KDL::Frame actual_ee;
        if (fk.JntToCart(solved, actual_ee) < 0) {
          fail("Waypoint FK failed");
          return;
        }
        const KDL::Frame actual_tcp = actual_ee * ee_tcp;
        const KDL::Twist err = KDL::diff(wanted_tcp, actual_tcp);
        const double pos_mm = err.vel.Norm() * 1e3;
        const double rot_rad = err.rot.Norm();
        if (!std::isfinite(pos_mm) || !std::isfinite(rot_rad) ||
            pos_mm > 0.5 || rot_rad > 0.002) {
          fail(std::string(stage.name) + " TCP FK tolerance exceeded");
          return;
        }
        max_fk_mm = std::max(max_fk_mm, pos_mm);
        max_fk_rad = std::max(max_fk_rad, rot_rad);

        KDL::JntArray mid_q(6);
        for (std::size_t j = 0; j < 6; ++j) {
          mid_q(j) = 0.5 * (previous_q(j) + solved(j));
        }
        KDL::Frame mid_ee;
        if (fk.JntToCart(mid_q, mid_ee) < 0) {
          fail("Midpoint FK failed");
          return;
        }
        const double mid_s = (static_cast<double>(i) - 0.5) /
          static_cast<double>(steps_per_stage_);
        const KDL::Frame mid_desired(initial_orientation,
          stage_start.p + delta_base * mid_s);
        const KDL::Twist mid_err = KDL::diff(mid_desired, mid_ee * ee_tcp);
        const double mid_mm = mid_err.vel.Norm() * 1e3;
        const double mid_rad = mid_err.rot.Norm();
        if (!std::isfinite(mid_mm) || !std::isfinite(mid_rad) ||
            mid_mm > 0.5 || mid_rad > 0.002) {
          fail(std::string(stage.name) + " joint interpolation midpoint tolerance exceeded");
          return;
        }
        max_mid_mm = std::max(max_mid_mm, mid_mm);
        max_mid_rad = std::max(max_mid_rad, mid_rad);

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(6);
        for (std::size_t j = 0; j < 6; ++j) point.positions[j] = solved(j);
        elapsed += dt;
        point.time_from_start = rclcpp::Duration::from_seconds(elapsed);
        trajectory.points.push_back(point);
        poses.poses.push_back(to_pose(wanted_tcp));
        previous_q = solved;
      }
      stage_start = stage_goal;
      summary << stage.name << " frame=" <<
        (stage.axes == Axes::TCP ? "tool_tcp" : "base_link")
        << " points=[" << begin_index << "," << trajectory.points.size() - 1
        << "] end_time_s=" << elapsed << " goal_base_xyz_m=["
        << stage_goal.p.x() << "," << stage_goal.p.y() << ","
        << stage_goal.p.z() << "]\n";
    }

    if (trajectory.points.size() != 1 + stages.size() *
        static_cast<std::size_t>(steps_per_stage_) ||
        poses.poses.size() != trajectory.points.size()) {
      fail("Internal stage/point count mismatch");
      return;
    }

    std_msgs::msg::String manifest;
    manifest.data = summary.str();
    // Only publish once the COMPLETE sequence has passed all checks.
    trajectory_pub_->publish(trajectory);
    pose_pub_->publish(poses);
    manifest_pub_->publish(manifest);
    planned_ = true;
    success_ = true;

    RCLCPP_INFO(get_logger(), "========== 5D SEQUENCE PREVIEW SUCCESS ==========");
    RCLCPP_INFO(get_logger(),
      "Six stages, %zu joint/TCP poses, %.2fs total; fixed TCP orientation",
      trajectory.points.size(), elapsed);
    RCLCPP_INFO(get_logger(),
      "Max TCP FK error: %.6f mm, %.6f rad; midpoint: %.6f mm, %.6f rad",
      max_fk_mm, max_fk_rad, max_mid_mm, max_mid_rad);
    RCLCPP_INFO(get_logger(),
      "Max joint step %.6f rad; max vel/URDF %.2f%%",
      max_step_rad, max_velocity_ratio * 100.0);
    RCLCPP_INFO(get_logger(),
      "Published: /mtc/sequence_preview /mtc/sequence_tcp_poses /mtc/sequence_stages");
    RCLCPP_WARN(get_logger(),
      "NO EXECUTION. No real ring/board frame. No collision/acceleration/dynamics validation.");
    RCLCPP_INFO(get_logger(), "Node alive for topic inspection; Ctrl+C to exit");
  }

  KDL::Chain chain_;
  KDL::JntArray current_q_, q_min_, q_max_;
  std::vector<std::string> joint_names_;
  std::vector<double> velocity_limits_;

  bool confirm_{false};
  double approach_m_{0.005}, insert_m_{0.010}, lift_m_{0.005};
  double transfer_x_m_{0.005}, lower_m_{0.005}, withdraw_m_{0.015};
  int steps_per_stage_{10};
  double seconds_per_stage_{2.0};
  double max_joint_step_{0.10}, tf_wait_seconds_{10.0};
  bool model_ready_{false}, state_received_{false};
  bool planned_{false}, success_{false};
  std::chrono::steady_clock::time_point started_{}, last_state_{};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manifest_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TcpSequencePlanner>();
  rclcpp::spin(node);
  const bool passed = node->passed();
  if (rclcpp::ok()) rclcpp::shutdown();
  return passed ? 0 : 1;
}
