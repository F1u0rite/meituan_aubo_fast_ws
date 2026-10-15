#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using Trajectory = trajectory_msgs::msg::JointTrajectory;
using PoseArray = geometry_msgs::msg::PoseArray;
using JointState = sensor_msgs::msg::JointState;

namespace {
const std::array<std::string, 6> kJoints = {
  "shoulder_joint", "upperArm_joint", "foreArm_joint",
  "wrist1_joint", "wrist2_joint", "wrist3_joint"
};
const std::array<std::string, 6> kStages = {
  "01_APPROACH_TCP_X", "02_INSERT_TCP_X", "03_LIFT_BASE_Z",
  "04_TRANSFER_BASE_X", "05_LOWER_BASE_Z", "06_WITHDRAW_TCP_X"
};
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kPoints = 61;
constexpr double kDt = 0.2;

double seconds(const trajectory_msgs::msg::JointTrajectoryPoint &p)
{
  return p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec;
}
bool same_stamp(const builtin_interfaces::msg::Time &a,
                const builtin_interfaces::msg::Time &b)
{
  return a.sec == b.sec && a.nanosec == b.nanosec;
}
KDL::Frame pose_to_kdl(const geometry_msgs::msg::Pose &p)
{
  const auto &q = p.orientation;
  return KDL::Frame(KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w),
                    KDL::Vector(p.position.x, p.position.y, p.position.z));
}
}

class TcpSequenceMonitor final : public rclcpp::Node
{
public:
  TcpSequenceMonitor()
    : Node("mtc_tcp_sequence_monitor")
  {
    confirm_ = declare_parameter<bool>("confirm_nominal_tool_tf", false);
    started_ = now();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) { read_urdf(msg); });
    joint_sub_ = create_subscription<JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const JointState::SharedPtr msg) { read_state(msg); });
    preview_sub_ = create_subscription<Trajectory>(
      "/mtc/sequence_preview", rclcpp::QoS(1).reliable().transient_local(),
      [this](const Trajectory::SharedPtr msg) { read_preview(msg); });
    poses_sub_ = create_subscription<PoseArray>(
      "/mtc/sequence_tcp_poses", rclcpp::QoS(1).reliable().transient_local(),
      [this](const PoseArray::SharedPtr msg) { poses_ = msg; });
    manifest_sub_ = create_subscription<std_msgs::msg::String>(
      "/mtc/sequence_stages", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) { manifest_ = msg; });
    armed_pub_ = create_publisher<std_msgs::msg::Header>(
      "/mtc/sequence_monitor_armed", rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(100ms, [this] { tick(); });
    RCLCPP_WARN(get_logger(),
      "5D-2 MONITOR: nominal V1 TCP, PREVIEW VALIDATION / Mock telemetry ONLY");
    RCLCPP_WARN(get_logger(),
      "No emergency stop or collision/acceleration/dynamics checks; never use on real hardware");
    if (!confirm_) {
      RCLCPP_ERROR(get_logger(),
        "Require -p confirm_nominal_tool_tf:=true to publish monitor arm");
    }
  }
  bool passed() const { return passed_; }

private:
  void finish(bool success, const std::string &message)
  {
    if (done_) return;
    done_ = true;
    passed_ = success;
    RCLCPP_INFO(get_logger(), "========== 5D-2 SEQUENCE MONITOR SUMMARY ==========");
    RCLCPP_INFO(get_logger(), "TCP samples=%zu; max path distance=%.6f mm",
      samples_, max_cross_m_ * 1e3);
    RCLCPP_INFO(get_logger(), "Max orientation error=%.8f rad; final TCP error=%.6f mm",
      max_orient_, final_err_m_ * 1e3);
    for (std::size_t i = 0; i < 6; ++i) {
      RCLCPP_INFO(get_logger(), "%s: samples=%zu max_path_error=%.6f mm",
        kStages[i].c_str(), stage_samples_[i], stage_max_m_[i] * 1e3);
    }
    if (success)
      RCLCPP_INFO(get_logger(), "5D-2 TCP SEQUENCE MONITOR SUCCESS: %s", message.c_str());
    else
      RCLCPP_ERROR(get_logger(), "5D-2 TCP SEQUENCE MONITOR FAILED: %s", message.c_str());
    rclcpp::shutdown();
  }

  void read_urdf(const std_msgs::msg::String::SharedPtr msg)
  {
    if (ready_model_ || done_) return;
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(msg->data, tree) ||
        !tree.getChain("base_link", "ee_link", chain_) ||
        chain_.getNrOfJoints() != 6) {
      finish(false, "Invalid base_link -> ee_link URDF/KDL chain");
      return;
    }
    std::size_t n = 0;
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
      const auto &joint = chain_.getSegment(i).getJoint();
      if (joint.getType() == KDL::Joint::None) continue;
      if (n >= 6 || joint.getName() != kJoints[n]) {
        finish(false, "KDL joint ordering differs from verified S3 model");
        return;
      }
      ++n;
    }
    if (n != 6) { finish(false, "Not six KDL joints"); return; }
    fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    ready_model_ = true;
    RCLCPP_INFO(get_logger(), "KDL ready; waiting for NEW 5D preview + pose array + manifest");
  }

  void read_preview(const Trajectory::SharedPtr msg)
  {
    if (pending_ || armed_ || done_) return;
    if (msg->header.stamp.sec <= 0 || msg->header.frame_id != "base_link") return;
    const double after_start = (rclcpp::Time(msg->header.stamp) - started_).seconds();
    const double age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
    if (after_start < -0.10 || age < -1.0 || age > 15.0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
        "Ignoring old/future sequence preview; restart 5D planner AFTER monitor starts");
      return;
    }
    pending_ = msg;
    pending_at_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Received NEW 5D sequence preview: stamp=%d.%09u",
      msg->header.stamp.sec, msg->header.stamp.nanosec);
  }

  void read_state(const JointState::SharedPtr msg)
  {
    if (done_) return;
    std::array<double, 6> next{};
    for (std::size_t j = 0; j < 6; ++j) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), kJoints[j]);
      if (it == msg->name.end()) return;
      const auto n = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
      if (n >= msg->position.size() || !std::isfinite(msg->position[n])) return;
      next[j] = msg->position[n];
    }
    current_ = next;
    state_seen_ = true;
    last_state_ = std::chrono::steady_clock::now();
    if (armed_) sample_live();
  }

  bool tcp_fk(const std::array<double, 6> &q, KDL::Frame &out) const
  {
    if (!fk_) return false;
    KDL::JntArray joints(6);
    for (std::size_t j = 0; j < 6; ++j) joints(j) = q[j];
    KDL::Frame ee;
    if (fk_->JntToCart(joints, ee) < 0) return false;
    out = ee * ee_tcp_;
    return true;
  }

  bool nominal_tf_ok()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("ee_link", "tool_tcp", tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return false;
    }
    const auto &t = tf.transform.translation;
    const auto &q = tf.transform.rotation;
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
        !std::isfinite(norm) || norm < 1e-9) return false;
    ee_tcp_ = KDL::Frame(
      KDL::Rotation::Quaternion(q.x/norm, q.y/norm, q.z/norm, q.w/norm),
      KDL::Vector(t.x, t.y, t.z));
    const KDL::Frame nominal(KDL::Rotation::RotX(kPi),
      KDL::Vector(0.036437755, 0.0, 0.115));
    const auto d = KDL::diff(nominal, ee_tcp_);
    return std::isfinite(d.vel.Norm()) && std::isfinite(d.rot.Norm()) &&
      d.vel.Norm() <= 0.001 && d.rot.Norm() <= 0.01;
  }

  bool validate_sequence()
  {
    if (!confirm_ || !pending_ || !poses_ || !manifest_ || !ready_model_ || !state_seen_) return false;
    if (pending_->points.size() != kPoints || poses_->poses.size() != kPoints ||
        pending_->joint_names.size() != 6 ||
        !std::equal(kJoints.begin(), kJoints.end(), pending_->joint_names.begin()) ||
        poses_->header.frame_id != "base_link" ||
        !same_stamp(pending_->header.stamp, poses_->header.stamp)) {
      finish(false, "Trajectory/PoseArray length, stamp, joint or frame mismatch");
      return false;
    }
    // String manifest includes a matching full nanosecond timestamp and
    // the six exact stage names and fixed index/time boundaries.
    const auto &m = manifest_->data;
    const auto &stamp = pending_->header.stamp;
    char stamp_text[64];
    std::snprintf(stamp_text, sizeof(stamp_text), "Stamp=%d.%09u", stamp.sec, stamp.nanosec);
    if (m.find(stamp_text) == std::string::npos ||
        m.find("5D PREVIEW ONLY / NOMINAL TCP / NO EXECUTION") == std::string::npos) {
      // Transient-local cached manifest from a previous run: wait briefly.
      return false;
    }
    for (std::size_t i = 0; i < kStages.size(); ++i) {
      const std::string marker = kStages[i] + " frame=";
      const std::string bounds = "points=[" + std::to_string(i*10) + "," +
        std::to_string((i+1)*10) + "]";
      if (m.find(marker) == std::string::npos || m.find(bounds) == std::string::npos) {
        finish(false, "Stage manifest lacks required name/point boundaries");
        return false;
      }
    }
    if (!nominal_tf_ok()) return false;
    if (std::chrono::steady_clock::now() - last_state_ > 500ms) return false;

    planned_tcp_.clear();
    std::array<double, 6> first_q{}, previous_q{};
    KDL::Frame initial_tcp;
    for (std::size_t i = 0; i < kPoints; ++i) {
      const auto &pt = pending_->points[i];
      if (pt.positions.size() != 6 || !pt.velocities.empty() ||
          !pt.accelerations.empty() || !pt.effort.empty() ||
          std::abs(seconds(pt) - static_cast<double>(i)*kDt) > 0.001) {
        finish(false, "Invalid 61-point/12-second sequence format");
        return false;
      }
      std::array<double, 6> q{};
      for (std::size_t j = 0; j < 6; ++j) {
        q[j] = pt.positions[j];
        if (!std::isfinite(q[j]) || std::abs(q[j]) > 6.4) {
          finish(false, "Invalid joint position");
          return false;
        }
        if (i && (std::abs(q[j] - previous_q[j]) > 0.03 ||
                 std::abs(q[j] - previous_q[j]) / kDt > 0.10)) {
          finish(false, "Excessive step/speed for mock sequence");
          return false;
        }
        if (i == 0 && std::abs(q[j] - current_[j]) > 0.005) {
          finish(false, "Current mock joint state differs from sequence start");
          return false;
        }
      }
      KDL::Frame actual;
      if (!tcp_fk(q, actual)) {
        finish(false, "Waypoint TCP FK failure");
        return false;
      }
      const auto wanted_pose = pose_to_kdl(poses_->poses[i]);
      const auto delta = KDL::diff(wanted_pose, actual);
      if (!std::isfinite(delta.vel.Norm()) || !std::isfinite(delta.rot.Norm()) ||
          delta.vel.Norm() > 0.0005 || delta.rot.Norm() > 0.002) {
        finish(false, "FK differs from matching PoseArray waypoint");
        return false;
      }
      if (i == 0) { first_q = q; initial_tcp = actual; }
      previous_q = q;
      planned_tcp_.push_back(actual);
    }
    (void)first_q;
    // Every stage must be a straight fixed-orientation TCP line.
    // Check stage direction and geometry independently of the planner log.
    for (std::size_t stage = 0; stage < 6; ++stage) {
      const auto &start = planned_tcp_[stage*10];
      const auto &goal = planned_tcp_[(stage+1)*10];
      const KDL::Vector delta = goal.p - start.p;
      const double length = delta.Norm();
      if (!std::isfinite(length) || length < 0.0001 || length > 0.02) {
        finish(false, "Stage length outside mock bounds");
        return false;
      }
      KDL::Vector expected_axis;
      switch(stage) {
        case 0: case 1: expected_axis = initial_tcp.M * KDL::Vector(1,0,0); break;
        case 2: expected_axis = KDL::Vector(0,0,1); break;
        case 3: expected_axis = KDL::Vector(1,0,0); break;
        case 4: expected_axis = KDL::Vector(0,0,-1); break;
        default: expected_axis = initial_tcp.M * KDL::Vector(-1,0,0); break;
      }
      if ((delta - expected_axis * length).Norm() > 0.0005 ||
          KDL::diff(initial_tcp, goal).rot.Norm() > 0.002) {
        finish(false, "Stage direction/orientation invalid");
        return false;
      }
      for (std::size_t j = 1; j <= 10; ++j) {
        const auto &p = planned_tcp_[stage*10+j];
        const auto expected = start.p + delta * (static_cast<double>(j)/10.0);
        if ((p.p - expected).Norm() > 0.0005 ||
            KDL::diff(initial_tcp, p).rot.Norm() > 0.002) {
          finish(false, "Waypoint deviates from its straight stage segment");
          return false;
        }
      }
    }
    const auto insert_length = (planned_tcp_[20].p - planned_tcp_[0].p).Norm();
    const auto withdrawal = (planned_tcp_[60].p - planned_tcp_[50].p).Norm();
    const auto lift = (planned_tcp_[30].p - planned_tcp_[20].p).Norm();
    const auto lower = (planned_tcp_[50].p - planned_tcp_[40].p).Norm();
    if (std::abs(insert_length - withdrawal) > 0.0005 ||
        std::abs(lift - lower) > 0.0005) {
      finish(false, "Demo approach/insert/withdraw or lift/lower do not cancel");
      return false;
    }
    RCLCPP_INFO(get_logger(), "5D-2 validation PASSED: 6 stages, 61 TCP samples, 12s");
    return true;
  }

  double distance_to_path(const KDL::Vector &p) const
  {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < planned_tcp_.size(); ++i) {
      const auto a = planned_tcp_[i-1].p;
      const auto segment = planned_tcp_[i].p - a;
      const double n2 = KDL::dot(segment, segment);
      if (n2 < 1e-14) continue;
      const double s = std::clamp(KDL::dot(p-a, segment)/n2, 0.0, 1.0);
      best = std::min(best, (p - (a+segment*s)).Norm());
    }
    return best;
  }

  void sample_live()
  {
    double departure = 0.0;
    for (std::size_t j = 0; j < 6; ++j) {
      departure = std::max(departure,
        std::abs(current_[j]-pending_->points.front().positions[j]));
    }
    const auto stamp = std::chrono::steady_clock::now();
    if (!moving_) {
      if (departure < 0.00015) return;
      moving_ = true;
      motion_since_ = stamp;
      RCLCPP_INFO(get_logger(), "5D-2 TCP mock motion detected");
    }
    KDL::Frame actual;
    if (!tcp_fk(current_, actual)) {
      finish(false, "Live TCP FK error");
      return;
    }
    const double cross = distance_to_path(actual.p);
    const double rot = KDL::diff(planned_tcp_.front(), actual).rot.Norm();
    if (!std::isfinite(cross) || !std::isfinite(rot) ||
        cross > 0.0008 || rot > 0.01) {
      finish(false, "Live TCP left nominal 5D path corridor");
      return;
    }
    max_cross_m_ = std::max(max_cross_m_, cross);
    max_orient_ = std::max(max_orient_, rot);
    const double elapsed = std::chrono::duration<double>(stamp - motion_since_).count();
    const std::size_t stage = std::min<std::size_t>(5,
      static_cast<std::size_t>(std::max(0.0, elapsed) / 2.0));
    stage_samples_[stage]++;
    stage_max_m_[stage] = std::max(stage_max_m_[stage], cross);
    ++samples_;

    const auto end = planned_tcp_.back();
    final_err_m_ = (actual.p - end.p).Norm();
    double joint_error = 0.0;
    for (std::size_t j=0; j<6; ++j) {
      joint_error = std::max(joint_error,
        std::abs(current_[j] - pending_->points.back().positions[j]));
    }
    const bool near = final_err_m_ < 0.0005 && joint_error < 0.005 && rot < 0.003;
    if (near && elapsed >= 10.5) {
      if (!near_since_) near_since_ = stamp;
      if (stamp - *near_since_ > 600ms) {
        bool have_stages = true;
        for (const auto n : stage_samples_) have_stages = have_stages && n > 0;
        finish(have_stages && samples_ >= 100 && max_cross_m_ <= 0.0008,
          "Six-stage nominal TCP path check completed (GenericSystem only)");
      }
    } else {
      near_since_.reset();
    }
  }

  void tick()
  {
    if (done_ || !confirm_) return;
    const auto steady = std::chrono::steady_clock::now();
    if (!armed_ && pending_ && ready_model_ && state_seen_) {
      if ((now() - rclcpp::Time(pending_->header.stamp)).seconds() > 15.0) {
        finish(false, "Sequence preview expired before arm; restart new Planner");
        return;
      }
      if (validate_sequence()) {
        armed_ = true;
        arm_since_ = steady;
        armed_pub_->publish(pending_->header);
        RCLCPP_INFO(get_logger(),
          "5D-2 MONITOR ARMED: matching new 6-stage preview %d.%09u",
          pending_->header.stamp.sec, pending_->header.stamp.nanosec);
      } else if (steady - pending_at_ > 8s && !done_) {
        finish(false, "Sequence validation timed out waiting for matching PoseArray/manifest/TF");
      }
    }
    if (!armed_) return;
    if (!moving_ && steady - arm_since_ > 20s) {
      finish(false, "No Mock joint motion within 20 seconds after arm");
    } else if (moving_ && steady - motion_since_ > 22s) {
      finish(false, "12-second sequence monitoring timed out");
    } else if (steady - last_state_ > 1s) {
      finish(false, "Joint-state updates lost while monitoring");
    }
  }

  bool confirm_{false};
  bool done_{false}, passed_{false}, ready_model_{false}, state_seen_{false};
  bool armed_{false}, moving_{false};
  std::size_t samples_{0};
  double max_cross_m_{0.}, max_orient_{0.}, final_err_m_{0.};
  std::array<std::size_t, 6> stage_samples_{};
  std::array<double, 6> stage_max_m_{};
  std::array<double, 6> current_{};
  std::optional<std::chrono::steady_clock::time_point> near_since_;
  std::chrono::steady_clock::time_point last_state_{}, pending_at_{}, arm_since_{}, motion_since_{};
  rclcpp::Time started_{0,0,RCL_ROS_TIME};
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  KDL::Frame ee_tcp_;
  std::vector<KDL::Frame> planned_tcp_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Trajectory::SharedPtr pending_;
  PoseArray::SharedPtr poses_;
  std_msgs::msg::String::SharedPtr manifest_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_, manifest_sub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr preview_sub_;
  rclcpp::Subscription<PoseArray>::SharedPtr poses_sub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr armed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TcpSequenceMonitor>();
  rclcpp::spin(node);
  const bool passed = node->passed();
  if (rclcpp::ok()) rclcpp::shutdown();
  return passed ? 0 : 1;
}
