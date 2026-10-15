#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using FollowJT = control_msgs::action::FollowJointTrajectory;
using ListHW = controller_manager_msgs::srv::ListHardwareComponents;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using Trajectory = trajectory_msgs::msg::JointTrajectory;
using JointState = sensor_msgs::msg::JointState;

// MTC 5D-2 MOCK-ONLY SEQUENCE EXECUTOR.
// Only the STRICT 5D six-stage 61 point / 12 second preview is accepted.
// A separate KDL monitor independently checks trajectory geometry &
// PoseArray/manifest before publishing the matching Header arm.
// No live-hardware motion path exists in this program: GenericSystem only.
namespace {
const std::array<std::string, 6> kJoints = {
  "shoulder_joint", "upperArm_joint", "foreArm_joint",
  "wrist1_joint", "wrist2_joint", "wrist3_joint"
};
const std::array<double, 6> kLower = {
  -6.283185307179586, -3.141592653589793, -2.87979326579064,
  -6.283185307179586, -6.283185307179586, -6.283185307179586
};
const std::array<double, 6> kUpper = {
  6.283185307179586, 3.141592653589793, 2.87979326579064,
  6.283185307179586, 6.283185307179586, 6.283185307179586
};

double point_seconds(const trajectory_msgs::msg::JointTrajectoryPoint &p)
{
  return static_cast<double>(p.time_from_start.sec) +
    static_cast<double>(p.time_from_start.nanosec) * 1e-9;
}

bool same_header(const std_msgs::msg::Header &a, const std_msgs::msg::Header &b)
{
  return a.stamp.sec == b.stamp.sec && a.stamp.nanosec == b.stamp.nanosec &&
    a.frame_id == b.frame_id;
}

bool validate(const Trajectory &t, const std::array<double, 6> &current,
              const rclcpp::Logger &log)
{
  if (t.header.frame_id != "base_link" || t.joint_names.size() != 6 ||
      !std::equal(kJoints.begin(), kJoints.end(), t.joint_names.begin()) ||
      t.points.size() != 61) {
    RCLCPP_ERROR(log, "5D-2: unexpected frame/joints/count; require 61 points");
    return false;
  }
  for (size_t i = 0; i < t.points.size(); ++i) {
    const auto &p = t.points[i];
    if (p.positions.size() != 6 || !p.velocities.empty() ||
        !p.accelerations.empty() || !p.effort.empty() ||
        std::abs(point_seconds(p) - 0.2 * static_cast<double>(i)) > 0.001) {
      RCLCPP_ERROR(log, "5D-2: wrong point format or 12s timing at %zu", i);
      return false;
    }
    for (size_t j = 0; j < 6; ++j) {
      const double q = p.positions[j];
      if (!std::isfinite(q) || q < kLower[j] + 0.01 ||
          q > kUpper[j] - 0.01 ||
          std::abs(q - t.points.front().positions[j]) > 0.30) {
        RCLCPP_ERROR(log, "5D-2: joint limit or excursion at %zu/%zu", i, j);
        return false;
      }
      if (i == 0 && std::abs(q - current[j]) > 0.005) {
        RCLCPP_ERROR(log, "5D-2: stale start at %s", kJoints[j].c_str());
        return false;
      }
      if (i > 0) {
        const double dq = std::abs(q - t.points[i-1].positions[j]);
        if (dq > 0.03 || dq / 0.2 > 0.10) {
          RCLCPP_ERROR(log, "5D-2: step/speed limit at %zu/%zu", i, j);
          return false;
        }
      }
    }
  }
  return true;
}

// Below is the already validated 5C V3 hardware/controller identity check.
// IMPORTANT: keep each service response shared_ptr alive while reading fields.
bool check_mock(const rclcpp::Node::SharedPtr &node)
{
  auto h = node->create_client<ListHW>(
    "/controller_manager/list_hardware_components");
  if (!h->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "No controller_manager hardware service");
    return false;
  }
  auto future = h->async_send_request(std::make_shared<ListHW::Request>());
  if (rclcpp::spin_until_future_complete(node, future, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Hardware query timeout");
    return false;
  }
  // Keep the service response shared_ptr alive for the whole inspection.
  // FutureAndRequestId::get() returns the shared_ptr by value in Jazzy;
  // taking a reference directly to get()->component would dangle.
  const auto hw_response = future.get();
  if (!hw_response) {
    RCLCPP_ERROR(node->get_logger(),
      "Null hardware service response: REFUSED (NO GOAL SENT)");
    return false;
  }
  const auto &parts = hw_response->component;

  // Diagnostic revision: show the *actual response observed by this process*.
  // Never infer Mock safety from a previous shell command: each check must
  // independently validate the live controller_manager service response.
  RCLCPP_INFO(node->get_logger(),
    "5C HW CHECK: received %zu component(s); require exactly 1",
    parts.size());

  for (size_t i = 0; i < parts.size(); ++i) {
    const auto &p = parts[i];
    const bool name_ok = (p.name == "auboHardwareInterface");
    const bool plugin_ok = (p.plugin_name == "mock_components/GenericSystem");
    const bool state_ok = (p.state.label == "active");

    RCLCPP_INFO(node->get_logger(),
      "5C HW[%zu]: name='%s' (length=%zu, match=%s); "
      "plugin='%s' (length=%zu, match=%s); "
      "state={id=%u, label='%s'} (label_match=%s)",
      i,
      p.name.c_str(), p.name.size(), name_ok ? "YES" : "NO",
      p.plugin_name.c_str(), p.plugin_name.size(),
      plugin_ok ? "YES" : "NO",
      static_cast<unsigned int>(p.state.id),
      p.state.label.c_str(), state_ok ? "YES" : "NO");
  }

  if (parts.size() != 1 || parts[0].name != "auboHardwareInterface" ||
      parts[0].plugin_name != "mock_components/GenericSystem" ||
      parts[0].state.label != "active") {
    RCLCPP_ERROR(node->get_logger(),
      "REAL/UNKNOWN HARDWARE: REFUSED (inspect HW CHECK above; "
      "NO GOAL SENT)");
    return false;
  }
  auto c = node->create_client<ListControllers>(
    "/controller_manager/list_controllers");
  if (!c->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "No controller list service");
    return false;
  }
  auto cf = c->async_send_request(std::make_shared<ListControllers::Request>());
  if (rclcpp::spin_until_future_complete(node, cf, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Controller query timeout");
    return false;
  }
  // Same lifetime rule for the controller service response.
  const auto ctrl_response = cf.get();
  if (!ctrl_response) {
    RCLCPP_ERROR(node->get_logger(),
      "Null controller service response: REFUSED (NO GOAL SENT)");
    return false;
  }
  bool jt = false, js = false;
  for (const auto &item : ctrl_response->controller) {
    if (item.name == "joint_trajectory_controller" && item.state == "active" &&
        item.type == "joint_trajectory_controller/JointTrajectoryController") {
      jt = true;
    }
    if (item.name == "joint_state_broadcaster" && item.state == "active") {
      js = true;
    }
  }
  if (!jt || !js) {
    RCLCPP_ERROR(node->get_logger(), "Required controllers not active");
    return false;
  }
  RCLCPP_INFO(node->get_logger(), "Verified GenericSystem and active controllers");
  return true;
}

int run(const rclcpp::Node::SharedPtr &node)
{
  const bool diagnostic = node->declare_parameter<bool>("diagnostic_only", false);
  if (diagnostic) {
    RCLCPP_WARN(node->get_logger(),
      "5D-2 DIAGNOSTIC ONLY: hardware/controller inspection, NO ACTION CLIENT");
    const bool ok = check_mock(node);
    RCLCPP_INFO(node->get_logger(), "5D-2 DIAG: %s (NO GOAL SENT)",
      ok ? "GENERIC SYSTEM VERIFIED" : "REFUSED");
    return ok ? 0 : 1;
  }
  const bool enable = node->declare_parameter<bool>("enable_mock_motion", false);
  const bool acknowledge = node->declare_parameter<bool>(
    "acknowledge_nominal_sequence", false);
  if (!enable || !acknowledge) {
    RCLCPP_ERROR(node->get_logger(),
      "LOCKED. Requires enable_mock_motion:=true AND "
      "acknowledge_nominal_sequence:=true (GenericSystem only)");
    return 1;
  }
  RCLCPP_WARN(node->get_logger(),
    "5D-2 MOCK ONLY: nominal V1 TCP, no collision/dynamics/acceleration checks");
  const rclcpp::Time started = node->now();
  if (!check_mock(node)) return 1;

  std::array<double, 6> joints{};
  bool state_ok = false;
  std::chrono::steady_clock::time_point last_state{};
  Trajectory::SharedPtr preview;
  std_msgs::msg::Header armed_header;
  bool monitor_armed = false;

  auto joint_sub = node->create_subscription<JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [&](const JointState::SharedPtr msg) {
      std::array<double, 6> q{};
      for (size_t j = 0; j < 6; ++j) {
        const auto it = std::find(msg->name.begin(), msg->name.end(), kJoints[j]);
        if (it == msg->name.end()) return;
        const auto idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
        if (idx >= msg->position.size() || !std::isfinite(msg->position[idx])) return;
        q[j] = msg->position[idx];
      }
      joints = q;
      state_ok = true;
      last_state = std::chrono::steady_clock::now();
    });
  auto preview_sub = node->create_subscription<Trajectory>(
    "/mtc/sequence_preview", rclcpp::QoS(1).reliable().transient_local(),
    [&](const Trajectory::SharedPtr msg) {
      const auto stamp = rclcpp::Time(msg->header.stamp);
      const double age = (node->now() - stamp).seconds();
      const double from_start = (stamp - started).seconds();
      if (msg->header.stamp.sec <= 0 || from_start < -0.10 ||
          age < -1.0 || age > 15.0) {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 3000,
          "Ignoring stale/future 5D preview: age=%.1fs", age);
        return;
      }
      if (preview) return;
      preview = msg;
      RCLCPP_INFO(node->get_logger(),
        "NEW 5D preview received. Waiting for MATCHING sequence monitor arm");
    });
  auto arm_sub = node->create_subscription<std_msgs::msg::Header>(
    "/mtc/sequence_monitor_armed", rclcpp::QoS(1).reliable().transient_local(),
    [&](const std_msgs::msg::Header::SharedPtr msg) {
      armed_header = *msg;
      monitor_armed = true;
    });

  RCLCPP_INFO(node->get_logger(),
    "5D-2 EXECUTOR ARMED TO WAIT (60s). Launch ONE fresh 5D planner now.");
  const auto deadline = std::chrono::steady_clock::now() + 60s;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    if (preview && monitor_armed && same_header(preview->header, armed_header) &&
        state_ok && std::chrono::steady_clock::now() - last_state < 500ms)
      break;
    std::this_thread::sleep_for(20ms);
  }
  if (!preview || !monitor_armed || !same_header(preview->header, armed_header) ||
      !state_ok || std::chrono::steady_clock::now() - last_state >= 500ms) {
    RCLCPP_ERROR(node->get_logger(),
      "No NEW preview + matching monitor arm + fresh joint state in 60 seconds");
    return 1;
  }
  const auto stamp = rclcpp::Time(preview->header.stamp);
  if ((node->now() - stamp).seconds() > 15.0 ||
      !validate(*preview, joints, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "5D-2 preview stale/invalid");
    return 1;
  }
  auto action = rclcpp_action::create_client<FollowJT>(
    node, "/joint_trajectory_controller/follow_joint_trajectory");
  if (!action->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node->get_logger(), "No FollowJointTrajectory server");
    return 1;
  }
  // Mandatory second identity check just before sending the only goal.
  if (!check_mock(node)) return 1;
  rclcpp::spin_some(node);
  if (!state_ok || std::chrono::steady_clock::now() - last_state > 500ms ||
      (node->now() - stamp).seconds() > 15.0 ||
      !validate(*preview, joints, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "5D-2 preview/current state changed: REFUSED");
    return 1;
  }
  FollowJT::Goal goal;
  goal.trajectory = *preview;
  goal.trajectory.header.stamp.sec = 0; // execute immediately in Mock
  goal.trajectory.header.stamp.nanosec = 0;
  auto options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
  options.feedback_callback = [node](auto,
      const std::shared_ptr<const FollowJT::Feedback> msg) {
    if (!msg->actual.positions.empty()) {
      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
        "5D-2 MOCK feedback: first joint=%.5f", msg->actual.positions.front());
    }
  };
  RCLCPP_WARN(node->get_logger(),
    "SENDING ONE 12s / 61-point 5D-2 SEQUENCE TO VERIFIED GENERICSYSTEM MOCK");
  auto goal_future = action->async_send_goal(goal, options);
  if (rclcpp::spin_until_future_complete(node, goal_future, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Goal acknowledgement timeout; state unknown");
    return 1;
  }
  const auto handle = goal_future.get();
  if (!handle) {
    RCLCPP_ERROR(node->get_logger(), "5D-2 Mock Action rejected");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "5D-2 MOCK goal ACCEPTED");
  auto result_future = action->async_get_result(handle);
  if (rclcpp::spin_until_future_complete(node, result_future, 25s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(),
      "Sequence result timeout; requesting cancel; do not assume stopped");
    auto cancel = action->async_cancel_goal(handle);
    (void)rclcpp::spin_until_future_complete(node, cancel, 3s);
    return 1;
  }
  const auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !result.result || result.result->error_code != FollowJT::Result::SUCCESSFUL) {
    RCLCPP_ERROR(node->get_logger(), "5D-2 Mock controller reported failure");
    return 1;
  }
  // Action success is not evidence of final robot pose; verify new feedback.
  const auto last = preview->points.back().positions;
  const auto deadline_final = std::chrono::steady_clock::now() + 2s;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline_final) {
    rclcpp::spin_some(node);
    if (state_ok && std::chrono::steady_clock::now() - last_state < 500ms) {
      double err = 0.0;
      for (size_t j = 0; j < 6; ++j)
        err = std::max(err, std::abs(joints[j] - last[j]));
      if (err <= 0.005) {
        RCLCPP_INFO(node->get_logger(),
          "5D-2 MOCK ACTION SUCCESS; final joint error=%.7f rad", err);
        return 0;
      }
    }
    std::this_thread::sleep_for(20ms);
  }
  RCLCPP_ERROR(node->get_logger(),
    "5D-2 Action finished but final joint feedback did not verify");
  return 1;
}
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mtc_tcp_sequence_executor");
  const int status = run(node);
  if (rclcpp::ok()) rclcpp::shutdown();
  return status;
}
