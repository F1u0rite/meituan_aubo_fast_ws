// 5E-2: READ-ONLY hypothetical TCP pose + IK branch scouting.
// No publishers, no action clients, no controller access. This is NOT a motion planner.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <urdf/model.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using JointState = sensor_msgs::msg::JointState;

class WorkspacePoseScout final : public rclcpp::Node
{
public:
  WorkspacePoseScout()
  : Node("mtc_workspace_pose_scout"), q_snapshot_(6), q_lower_(6), q_upper_(6)
  {
    acknowledged_ = declare_parameter<bool>("confirm_nominal_tool_tf", false);
    offsets_deg_ = declare_parameter<std::vector<double>>(
      "pitch_offsets_deg", {-15.0, 0.0, 15.0, 30.0, 38.4});
    transfers_m_ = declare_parameter<std::vector<double>>(
      "transfer_distances_m", {0.010, 0.020, 0.050});
    approach_m_ = declare_parameter<double>("approach_m", 0.001);
    insert_m_ = declare_parameter<double>("insert_m", 0.002);
    lift_m_ = declare_parameter<double>("lift_m", 0.005);
    lower_m_ = declare_parameter<double>("lower_m", 0.005);
    withdraw_m_ = declare_parameter<double>("withdraw_m", 0.003);
    steps_ = declare_parameter<int>("steps_per_stage", 20);
    sec_per_stage_ = declare_parameter<double>("seconds_per_stage", 2.0);
    max_step_ = declare_parameter<double>("max_joint_step_rad", 0.10);
    cond_heuristic_ = declare_parameter<double>("condition_heuristic", 150.0);
    jac_scale_ = declare_parameter<double>("jacobian_scale_m", 0.30);
    timeout_s_ = declare_parameter<double>("startup_timeout_s", 20.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) { on_urdf(msg); });
    joint_sub_ = create_subscription<JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const JointState::SharedPtr msg) { on_joint(msg); });
    start_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(200ms, [this] { tick(); });
    RCLCPP_WARN(get_logger(),
      "5E-2 READ ONLY: NO ROS motion/control publisher, NO controller calls, NO robot movement");
    RCLCPP_WARN(get_logger(),
      "Hypothetical START POSES only; nominal uncalibrated TCP, no collision/approach-transition checks");
  }

private:
  struct Stage {
    const char *label;
    bool tcp_local;
    KDL::Vector shift;
  };

  struct Result {
    bool pass{false};
    std::string reason;
    std::string stage;
    int index{0};
    double smallest_sigma{std::numeric_limits<double>::infinity()};
    double greatest_condition{0.};
    double smallest_margin{std::numeric_limits<double>::infinity()};
    double max_step{0.};
    double max_velocity{0.};
    double max_fk_mm{0.};
    double max_mid_mm{0.};
  };

  void fail(const std::string &message)
  {
    if (done_) return;
    RCLCPP_ERROR(get_logger(), "5E-2 SETUP ERROR: %s", message.c_str());
    done_ = true;
    rclcpp::shutdown();
  }

  static bool positive(double v) { return std::isfinite(v) && v > 0.; }

  bool validate_parameters()
  {
    if (!acknowledged_) { fail("Require -p confirm_nominal_tool_tf:=true"); return false; }
    if (offsets_deg_.empty() || offsets_deg_.size() > 15 ||
        transfers_m_.empty() || transfers_m_.size() > 8 ||
        steps_ < 5 || steps_ > 80 || !positive(sec_per_stage_) ||
        sec_per_stage_ > 30. || !positive(max_step_) || max_step_ > 0.2 ||
        !positive(jac_scale_) || jac_scale_ > 2. ||
        !positive(cond_heuristic_) || !positive(timeout_s_) || timeout_s_ > 120.) {
      fail("Invalid angle/distance list, steps or threshold parameters"); return false;
    }
    for (double d : offsets_deg_) {
      if (!std::isfinite(d) || std::abs(d) > 75.) {
        fail("Each pitch offset must be finite and within +/-75 degrees"); return false;
      }
    }
    for (double d : transfers_m_) {
      if (!positive(d) || d > 0.20) {
        fail("Transfer distance must be in (0, 0.20] m for offline scouting"); return false;
      }
    }
    for (double d : {approach_m_, insert_m_, lift_m_, lower_m_, withdraw_m_}) {
      if (!positive(d) || d > 0.020) {
        fail("Demo distance must be in (0, 0.020] m"); return false;
      }
    }
    return true;
  }

  void on_urdf(const std_msgs::msg::String::SharedPtr msg)
  {
    if (ready_urdf_ || done_) return;
    urdf::Model model;
    KDL::Tree tree;
    if (!model.initString(msg->data) ||
        !kdl_parser::treeFromString(msg->data, tree) ||
        !tree.getChain("base_link", "ee_link", chain_) || chain_.getNrOfJoints() != 6) {
      fail("Cannot build six-DOF base_link -> ee_link KDL chain"); return;
    }
    const std::array<std::string, 6> names = {{
      "shoulder_joint", "upperArm_joint", "foreArm_joint",
      "wrist1_joint", "wrist2_joint", "wrist3_joint"}};
    std::size_t i = 0;
    for (unsigned j = 0; j < chain_.getNrOfSegments(); ++j) {
      const auto &joint = chain_.getSegment(j).getJoint();
      if (joint.getType() == KDL::Joint::None) continue;
      if (i >= names.size() || joint.getName() != names[i]) {
        fail("Unexpected KDL joint order: " + joint.getName()); return;
      }
      const auto u = model.getJoint(names[i]);
      if (!u || u->type != urdf::Joint::REVOLUTE || !u->limits ||
          !std::isfinite(u->limits->lower) || !std::isfinite(u->limits->upper) ||
          u->limits->lower >= u->limits->upper || !positive(u->limits->velocity)) {
        fail("Invalid revolute joint limits: " + names[i]); return;
      }
      q_lower_(i) = u->limits->lower;
      q_upper_(i) = u->limits->upper;
      v_limits_.push_back(u->limits->velocity);
      joint_names_.push_back(names[i]);
      ++i;
    }
    if (i != 6) { fail("Expected exactly six revolute joints"); return; }
    ready_urdf_ = true;
    RCLCPP_INFO(get_logger(), "5E-2 KDL ready: %u segments, six joints",
      chain_.getNrOfSegments());
  }

  void on_joint(const JointState::SharedPtr msg)
  {
    if (!ready_urdf_ || done_) return;
    KDL::JntArray q(6);
    for (std::size_t j = 0; j < 6; ++j) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[j]);
      if (it == msg->name.end()) return;
      const auto idx = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
      if (idx >= msg->position.size()) return;
      const double v = msg->position[idx];
      if (!std::isfinite(v) || v < q_lower_(j) || v > q_upper_(j)) return;
      q(j) = v;
    }
    q_snapshot_ = q;
    ready_joint_ = true;
    last_joint_ = std::chrono::steady_clock::now();
  }

  bool get_nominal_tcp(KDL::Frame &ee_tcp)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform("ee_link", "tool_tcp", tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
        "Waiting for ee_link -> tool_tcp: %s", e.what());
      return false;
    }
    const auto &t = transform.transform.translation;
    const auto &q = transform.transform.rotation;
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (!std::isfinite(norm) || norm < 1e-9 ||
        !std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z)) {
      fail("Invalid tool_tcp TF"); return false;
    }
    ee_tcp = KDL::Frame(KDL::Rotation::Quaternion(
        q.x/norm, q.y/norm, q.z/norm, q.w/norm), KDL::Vector(t.x, t.y, t.z));
    constexpr double pi = 3.14159265358979323846;
    const KDL::Frame nominal(KDL::Rotation::RotX(pi),
                             KDL::Vector(0.036437755, 0., 0.115));
    const auto delta = KDL::diff(nominal, ee_tcp);
    if (delta.vel.Norm() > 0.001 || delta.rot.Norm() > 0.01) {
      fail("Current TCP TF differs from 5D nominal convention"); return false;
    }
    return true;
  }

  static std::string print_q(const KDL::JntArray &q)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(5) << "[";
    for (unsigned j = 0; j < q.rows(); ++j) {
      if (j) stream << ",";
      stream << q(j);
    }
    stream << "]";
    return stream.str();
  }

  double margin(const KDL::JntArray &q) const
  {
    double m = std::numeric_limits<double>::infinity();
    for (unsigned j = 0; j < 6; ++j) {
      m = std::min(m, std::min(q(j) - q_lower_(j), q_upper_(j) - q(j)));
    }
    return m;
  }

  bool pose_ok(const KDL::JntArray &q, const KDL::Frame &goal_ee,
               KDL::ChainFkSolverPos_recursive &fk, double &pos_mm,
               double &rotation_rad) const
  {
    for (unsigned j = 0; j < 6; ++j) {
      if (!std::isfinite(q(j)) || q(j) < q_lower_(j) || q(j) > q_upper_(j)) return false;
    }
    KDL::Frame got;
    if (fk.JntToCart(q, got) < 0) return false;
    const auto delta = KDL::diff(goal_ee, got);
    pos_mm = delta.vel.Norm() * 1000.;
    rotation_rad = delta.rot.Norm();
    return std::isfinite(pos_mm) && std::isfinite(rotation_rad) &&
      pos_mm <= 0.5 && rotation_rad <= 0.002;
  }

  std::pair<double, double> jac_metrics(const KDL::JntArray &q,
    const KDL::Frame &ee_tcp, KDL::ChainFkSolverPos_recursive &fk,
    KDL::ChainJntToJacSolver &jac_solver) const
  {
    KDL::Frame ee;
    KDL::Jacobian jac(6);
    if (fk.JntToCart(q, ee) < 0 || jac_solver.JntToJac(q, jac) < 0) {
      return {0.0, std::numeric_limits<double>::infinity()};
    }
    jac.changeRefPoint(ee.M * ee_tcp.p);
    Eigen::Matrix<double, 6, 6> scaled = jac.data;
    scaled.bottomRows<3>() *= jac_scale_;
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(scaled);
    const auto s = svd.singularValues();
    const double sigma = s(5);
    return {sigma, sigma < 1e-12 ? std::numeric_limits<double>::infinity() : s(0)/sigma};
  }

  std::vector<KDL::JntArray> make_seeds(const KDL::JntArray &current) const
  {
    std::vector<KDL::JntArray> seeds;
    seeds.push_back(current);
    // Deterministic local and elbow-branch probes. Not necessarily connected
    // by a collision-free trajectory to the actual robot's present posture.
    const std::array<std::pair<std::size_t, double>, 10> variants = {{
      {2, -0.7}, {2, 0.7}, {1, -0.6}, {1, 0.6},
      {3, -0.5}, {3, 0.5}, {4, -0.45}, {4, 0.45},
      {0, -0.5}, {0, 0.5}}};
    for (auto v : variants) {
      KDL::JntArray q = current;
      q(v.first) = std::min(q_upper_(v.first)-0.02,
                            std::max(q_lower_(v.first)+0.02, current(v.first)+v.second));
      seeds.push_back(q);
    }
    // A larger elbow/shoulder mirror seed may reach another IK branch.
    KDL::JntArray opposite = current;
    opposite(2) = std::max(q_lower_(2)+0.02,
      std::min(q_upper_(2)-0.02, -current(2)-0.4));
    opposite(1) = std::max(q_lower_(1)+0.02,
      std::min(q_upper_(1)-0.02, current(1)+0.7));
    seeds.push_back(opposite);
    return seeds;
  }

  static double max_q_difference(const KDL::JntArray &a, const KDL::JntArray &b)
  {
    double m = 0.;
    for (unsigned j = 0; j < 6; ++j) m = std::max(m, std::abs(a(j)-b(j)));
    return m;
  }

  Result run_path(const KDL::JntArray &virtual_start, const KDL::Frame &ee_tcp,
                  double transfer, KDL::ChainFkSolverPos_recursive &fk,
                  KDL::ChainJntToJacSolver &jac_solver)
  {
    Result r;
    KDL::ChainIkSolverVel_pinv velocity_solver(chain_);
    KDL::ChainIkSolverPos_NR_JL ik(chain_, q_lower_, q_upper_, fk, velocity_solver, 300, 1e-6);
    const std::array<Stage, 6> stages = {{
      {"APPROACH", true,  KDL::Vector(approach_m_, 0., 0.)},
      {"INSERT", true,    KDL::Vector(insert_m_, 0., 0.)},
      {"LIFT", false,     KDL::Vector(0., 0., lift_m_)},
      {"TRANSFER", false, KDL::Vector(transfer, 0., 0.)},
      {"LOWER", false,    KDL::Vector(0., 0., -lower_m_)},
      {"WITHDRAW", true,  KDL::Vector(-withdraw_m_, 0., 0.)}}};
    KDL::Frame initial_ee;
    if (fk.JntToCart(virtual_start, initial_ee) < 0) { r.reason = "Start FK failed"; return r; }
    KDL::Frame seg_start = initial_ee * ee_tcp;
    const KDL::Rotation constant_orientation = seg_start.M;
    KDL::JntArray previous = virtual_start;
    const double dt = sec_per_stage_ / steps_;
    auto [sigma0, condition0] = jac_metrics(previous, ee_tcp, fk, jac_solver);
    r.smallest_sigma = sigma0;
    r.greatest_condition = condition0;
    r.smallest_margin = margin(previous);
    for (const auto &stage : stages) {
      const KDL::Vector delta = stage.tcp_local ? seg_start.M * stage.shift : stage.shift;
      for (int n = 1; n <= steps_; ++n) {
        const double fraction = static_cast<double>(n) / steps_;
        const KDL::Frame wanted_tcp(
          constant_orientation, seg_start.p + delta*fraction);
        const KDL::Frame wanted_ee = wanted_tcp * ee_tcp.Inverse();
        KDL::JntArray out(6);
        const int code = ik.CartToJnt(previous, wanted_ee, out);
        r.stage = stage.label;
        r.index = n;
        if (code < 0) { r.reason = "IK code="+std::to_string(code); return r; }
        double fk_mm = 0., rot = 0.;
        if (!pose_ok(out, wanted_ee, fk, fk_mm, rot)) {
          r.reason = "FK / limits gate"; return r;
        }
        r.max_fk_mm = std::max(r.max_fk_mm, fk_mm);
        r.smallest_margin = std::min(r.smallest_margin, margin(out));
        for (unsigned j = 0; j < 6; ++j) {
          const double change = std::abs(out(j)-previous(j));
          r.max_step = std::max(r.max_step, change);
          r.max_velocity = std::max(r.max_velocity, change/dt/v_limits_[j]);
          if (change > max_step_ || change/dt/v_limits_[j] > 0.5) {
            r.reason = "Joint step / speed gate"; return r;
          }
        }
        KDL::JntArray midpoint(6);
        for (unsigned j=0; j<6; ++j) midpoint(j) = 0.5*(out(j)+previous(j));
        KDL::Frame mid_ee;
        if (fk.JntToCart(midpoint, mid_ee) < 0) { r.reason = "Mid FK"; return r; }
        const KDL::Frame mid_desired(constant_orientation,
          seg_start.p+delta*((static_cast<double>(n)-0.5)/steps_));
        const auto mid_error = KDL::diff(mid_desired, mid_ee*ee_tcp);
        const double mid_mm = mid_error.vel.Norm()*1000.;
        r.max_mid_mm = std::max(r.max_mid_mm, mid_mm);
        if (!std::isfinite(mid_mm) || mid_mm > 0.5 || mid_error.rot.Norm() > 0.002) {
          r.reason = "Midpoint TCP gate"; return r;
        }
        auto [sigma, condition] = jac_metrics(out, ee_tcp, fk, jac_solver);
        r.smallest_sigma = std::min(r.smallest_sigma, sigma);
        r.greatest_condition = std::max(r.greatest_condition, condition);
        previous = out;
      }
      seg_start = KDL::Frame(constant_orientation, seg_start.p+delta);
    }
    r.pass = true;
    r.reason = "Geometric IK passed; no start-transition, collision or dynamics validation";
    return r;
  }

  void scout(const KDL::Frame &ee_tcp)
  {
    KDL::ChainFkSolverPos_recursive fk(chain_);
    KDL::ChainJntToJacSolver jac_solver(chain_);
    KDL::ChainIkSolverVel_pinv vel_solver(chain_);
    KDL::ChainIkSolverPos_NR_JL init_ik(
      chain_, q_lower_, q_upper_, fk, vel_solver, 700, 1e-6);
    const KDL::JntArray snapshot = q_snapshot_;
    KDL::Frame start_ee;
    if (fk.JntToCart(snapshot, start_ee) < 0) { fail("Cannot FK initial joints"); return; }
    const KDL::Frame tcp = start_ee * ee_tcp;
    const KDL::Vector nominal_axis = tcp.M * KDL::Vector(1., 0., 0.);
    const double current_insertion_pitch =
      std::atan2(nominal_axis.z(), std::hypot(nominal_axis.x(), nominal_axis.y())) *
      180.0 / 3.14159265358979323846;
    const auto seeds = make_seeds(snapshot);
    RCLCPP_INFO(get_logger(), "========== 5E-2 READ-ONLY POSE SCOUT ==========");
    RCLCPP_INFO(get_logger(), "Snapshot joints rad=%s", print_q(snapshot).c_str());
    RCLCPP_INFO(get_logger(), "Snapshot TCP xyz=[%.6f,%.6f,%.6f] m",
      tcp.p.x(), tcp.p.y(), tcp.p.z());
    RCLCPP_INFO(get_logger(),
      "Original TCP+X pitch=%.2f deg; offsets rotate target about BASE Y; +%.2f deg approximately levels TCP+X",
      current_insertion_pitch, current_insertion_pitch);
    RCLCPP_WARN(get_logger(),
      "Each candidate holds START TCP xyz fixed but changes its orientation: NOT the same ring insertion geometry");
    RCLCPP_WARN(get_logger(),
      "Candidate start joints are HYPOTHETICAL, not physically commanded or connected from current joints");
    std::size_t total_start = 0, total_path_pass = 0, total_condition_pass = 0;
    for (double offset : offsets_deg_) {
      constexpr double pi = 3.14159265358979323846;
      const double radians = offset*pi/180.;
      const KDL::Frame desired_tcp(KDL::Rotation::RotY(radians)*tcp.M, tcp.p);
      const KDL::Frame desired_ee = desired_tcp*ee_tcp.Inverse();
      const auto x_axis = desired_tcp.M*KDL::Vector(1.,0.,0.);
      RCLCPP_INFO(get_logger(),
        "========== PITCH base_Y %+6.2f deg | insertion axis=[%+.4f,%+.4f,%+.4f] ==========",
        offset, x_axis.x(), x_axis.y(), x_axis.z());
      std::vector<KDL::JntArray> solutions;
      for (const auto &seed : seeds) {
        KDL::JntArray q(6);
        const int code = init_ik.CartToJnt(seed, desired_ee, q);
        double pos_mm = 0., rot_rad = 0.;
        if (code < 0 || !pose_ok(q, desired_ee, fk, pos_mm, rot_rad)) continue;
        bool duplicate = false;
        for (const auto &old : solutions) {
          if (max_q_difference(q, old) < 0.10) { duplicate = true; break; }
        }
        if (!duplicate) solutions.push_back(q);
        if (solutions.size() >= 3) break;
      }
      if (solutions.empty()) {
        RCLCPP_WARN(get_logger(), "START_POSE_FAIL: no converged bounded IK from %zu deterministic seeds", seeds.size());
        continue;
      }
      RCLCPP_INFO(get_logger(), "START_POSE_PASS: %zu distinct IK branch(es) found", solutions.size());
      for (std::size_t branch=0; branch<solutions.size(); ++branch) {
        ++total_start;
        const auto &q = solutions[branch];
        RCLCPP_INFO(get_logger(), "Candidate P%+.1f/B%zu joints=%s start_margin=%.3f rad",
          offset, branch, print_q(q).c_str(), margin(q));
        for (double transfer : transfers_m_) {
          const auto outcome = run_path(q, ee_tcp, transfer, fk, jac_solver);
          if (outcome.pass) {
            ++total_path_pass;
            const bool favorable =
              std::isfinite(outcome.greatest_condition) &&
              outcome.greatest_condition <= cond_heuristic_;
            if (favorable) ++total_condition_pass;
            RCLCPP_INFO(get_logger(),
              "  P%+.1f/B%zu transfer=%5.1fmm PASS | heuristic=%s | min_sigma=%.6g "
              "max_cond=%.1f | min_margin=%.3f rad max_dq=%.4f rad vel=%.1f%% "
              "FK=%.4fmm midpoint=%.4fmm",
              offset, branch, transfer*1000., favorable ? "BETTER" : "CAUTION",
              outcome.smallest_sigma, outcome.greatest_condition,
              outcome.smallest_margin, outcome.max_step, outcome.max_velocity*100.,
              outcome.max_fk_mm, outcome.max_mid_mm);
          } else {
            RCLCPP_WARN(get_logger(),
              "  P%+.1f/B%zu transfer=%5.1fmm FAIL @%s point=%d: %s | "
              "min_sigma=%.6g max_cond=%.1f",
              offset, branch, transfer*1000., outcome.stage.c_str(),
              outcome.index, outcome.reason.c_str(),
              outcome.smallest_sigma, outcome.greatest_condition);
          }
        }
      }
    }
    RCLCPP_INFO(get_logger(),
      "5E-2 SUMMARY: hypothetical_start_poses=%zu, six_stage_path_pass=%zu, "
      "condition_below_%.0f=%zu",
      total_start, total_path_pass, cond_heuristic_, total_condition_pass);
    RCLCPP_WARN(get_logger(),
      "Heuristic != safety: no obstacles, gravity, acceleration, joint approach, "
      "verified ring pose, physical TCP calibration or contact checks");
    RCLCPP_INFO(get_logger(), "5E-2 READ-ONLY COMPLETE (NO GOAL SENT)");
    done_ = true;
    rclcpp::shutdown();
  }

  void tick()
  {
    if (done_ || !validate_parameters()) return;
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - start_).count() > timeout_s_) {
      fail("Timed out waiting for fresh /joint_states, /robot_description and TF"); return;
    }
    if (!ready_joint_ || !ready_urdf_) return;
    if (std::chrono::duration<double>(now-last_joint_).count() > 1.0) return;
    KDL::Frame ee_tcp;
    if (!get_nominal_tcp(ee_tcp)) return;
    timer_->cancel();
    scout(ee_tcp);
  }

  bool acknowledged_{false}, ready_urdf_{false}, ready_joint_{false}, done_{false};
  std::vector<double> offsets_deg_, transfers_m_, v_limits_;
  std::vector<std::string> joint_names_;
  double approach_m_{}, insert_m_{}, lift_m_{}, lower_m_{}, withdraw_m_{};
  int steps_{};
  double sec_per_stage_{}, max_step_{}, cond_heuristic_{}, jac_scale_{}, timeout_s_{};
  KDL::Chain chain_;
  KDL::JntArray q_snapshot_, q_lower_, q_upper_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point start_{}, last_joint_{};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WorkspacePoseScout>();
  rclcpp::spin(node);
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
