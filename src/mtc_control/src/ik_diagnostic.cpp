#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
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
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
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
#include <vector>
#include <utility>

using namespace std::chrono_literals;
using JointState = sensor_msgs::msg::JointState;

// 5E-1 read-only diagnostic. No publisher, no controller client, no action client.
// The reference trajectory construction matches 5D's fixed-orientation
// six stages, including TCP-local versus base-local displacement semantics.
class IkDiagnostic final : public rclcpp::Node
{
public:
  IkDiagnostic()
  : Node("mtc_ik_diagnostic"), snapshot_(6), lower_(6), upper_(6)
  {
    acknowledged_ = declare_parameter<bool>("confirm_nominal_tool_tf", false);
    approach_m_ = declare_parameter<double>("approach_m", 0.002);
    insert_m_ = declare_parameter<double>("insert_m", 0.005);
    lift_m_ = declare_parameter<double>("lift_m", 0.005);
    lower_m_ = declare_parameter<double>("lower_m", 0.005);
    withdraw_m_ = declare_parameter<double>("withdraw_m", 0.007);
    transfer_distances_ = declare_parameter<std::vector<double>>(
      "transfer_distances_m", {0.001, 0.005, 0.010, 0.020});
    steps_ = declare_parameter<int>("steps_per_stage", 10);
    seconds_per_stage_ = declare_parameter<double>("seconds_per_stage", 2.0);
    max_joint_step_ = declare_parameter<double>("max_joint_step", 0.10);
    characteristic_length_m_ = declare_parameter<double>("jacobian_scale_m", 0.30);
    timeout_s_ = declare_parameter<double>("startup_timeout_s", 20.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr m) { on_urdf(m); });
    joint_sub_ = create_subscription<JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const JointState::SharedPtr m) { on_joint_state(m); });
    started_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(200ms, [this]() { tick(); });
    RCLCPP_WARN(get_logger(),
      "5E-1 READ-ONLY IK DIAGNOSTIC: NO TRAJECTORY PUBLICATION, NO ACTION/ROBOT MOTION");
    RCLCPP_WARN(get_logger(),
      "Nominal, UNCALIBRATED V1 TCP. Use only to analyze current Mock configuration.");
  }

  bool completed() const { return completed_; }

private:
  struct Stage
  {
    const char *name;
    bool tcp_local;
    KDL::Vector delta;
  };

  struct JacobianMetrics
  {
    bool valid{false};
    double sigma_min{std::numeric_limits<double>::quiet_NaN()};
    double sigma_max{std::numeric_limits<double>::quiet_NaN()};
    double condition{std::numeric_limits<double>::infinity()};
    std::array<double, 6> singular{};
  };

  struct CaseResult
  {
    bool passed{false};
    std::string reason;
    std::string stage;
    int step{0};
    int ik_code{0};
    double max_joint_step{0.};
    double max_velocity_fraction{0.};
    double min_joint_margin_rad{std::numeric_limits<double>::infinity()};
    double smallest_sigma{std::numeric_limits<double>::infinity()};
    double largest_condition{0.};
    double max_fk_mm{0.};
    double max_midpoint_mm{0.};
  };

  void stop_with_error(const std::string &why)
  {
    if (finished_) return;
    RCLCPP_ERROR(get_logger(), "5E-1 SETUP FAILED: %s", why.c_str());
    finished_ = true;
    completed_ = false;
    rclcpp::shutdown();
  }

  static bool is_finite_positive(double v) { return std::isfinite(v) && v > 0.; }

  bool params_ok()
  {
    if (!acknowledged_) {
      stop_with_error("Pass -p confirm_nominal_tool_tf:=true for read-only nominal-model analysis");
      return false;
    }
    const std::array<double, 5> distances = {
      approach_m_, insert_m_, lift_m_, lower_m_, withdraw_m_};
    for (double x : distances) {
      if (!is_finite_positive(x) || x > 0.020) {
        stop_with_error("Demo stage distance must be >0 and <=0.020 m");
        return false;
      }
    }
    if (transfer_distances_.empty() || transfer_distances_.size() > 8) {
      stop_with_error("transfer_distances_m must have 1-8 values");
      return false;
    }
    for (double x : transfer_distances_) {
      if (!is_finite_positive(x) || x > 0.100) {
        stop_with_error("Each diagnostic transfer must be >0 and <=0.100 m");
        return false;
      }
    }
    if (steps_ < 5 || steps_ > 40 || !is_finite_positive(seconds_per_stage_) ||
        seconds_per_stage_ > 20. || !is_finite_positive(max_joint_step_) ||
        max_joint_step_ > 0.2 || !is_finite_positive(characteristic_length_m_) ||
        characteristic_length_m_ > 2. || !is_finite_positive(timeout_s_) ||
        timeout_s_ > 120.) {
      stop_with_error("Invalid steps/time/step-limit/Jacobian-scale/timeout parameters");
      return false;
    }
    return true;
  }

  void on_urdf(const std_msgs::msg::String::SharedPtr msg)
  {
    if (model_ready_ || finished_) return;
    urdf::Model model;
    KDL::Tree tree;
    if (!model.initString(msg->data) ||
        !kdl_parser::treeFromString(msg->data, tree) ||
        !tree.getChain("base_link", "ee_link", chain_) ||
        chain_.getNrOfJoints() != 6) {
      stop_with_error("Cannot build six-DOF KDL base_link -> ee_link chain");
      return;
    }
    const std::array<std::string, 6> expected = {{
      "shoulder_joint", "upperArm_joint", "foreArm_joint",
      "wrist1_joint", "wrist2_joint", "wrist3_joint"}};
    unsigned int n = 0;
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
      const auto &joint = chain_.getSegment(i).getJoint();
      if (joint.getType() == KDL::Joint::None) continue;
      const std::string name = joint.getName();
      if (n >= expected.size() || name != expected[n]) {
        stop_with_error("Unexpected KDL joint order: " + name);
        return;
      }
      const auto uj = model.getJoint(name);
      if (!uj || uj->type != urdf::Joint::REVOLUTE || !uj->limits) {
        stop_with_error("Missing revolute URDF joint or limits: " + name);
        return;
      }
      const double lo = uj->limits->lower;
      const double hi = uj->limits->upper;
      const double vel = uj->limits->velocity;
      if (!std::isfinite(lo) || !std::isfinite(hi) || lo >= hi ||
          !is_finite_positive(vel)) {
        stop_with_error("Invalid URDF limits: " + name);
        return;
      }
      joint_names_.push_back(name);
      lower_(n) = lo;
      upper_(n) = hi;
      velocity_limits_.push_back(vel);
      ++n;
    }
    if (n != 6) {
      stop_with_error("Expected six named revolute joints");
      return;
    }
    model_ready_ = true;
    RCLCPP_INFO(get_logger(), "5E-1 KDL ready: %u segments, 6 joints",
      chain_.getNrOfSegments());
  }

  void on_joint_state(const JointState::SharedPtr msg)
  {
    if (!model_ready_ || finished_) return;
    KDL::JntArray q(6);
    for (std::size_t i = 0; i < 6; ++i) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) return;
      const auto idx = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
      if (idx >= msg->position.size()) return;
      const double value = msg->position[idx];
      if (!std::isfinite(value) || value < lower_(i) || value > upper_(i)) return;
      q(i) = value;
    }
    snapshot_ = q;
    state_ready_ = true;
    last_state_ = std::chrono::steady_clock::now();
  }

  bool read_nominal_tf(KDL::Frame &ee_tcp)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        "ee_link", "tool_tcp", tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
        "Waiting for ee_link -> tool_tcp TF: %s", e.what());
      return false;
    }
    const auto &t = transform.transform.translation;
    const auto &q = transform.transform.rotation;
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
        !std::isfinite(norm) || norm < 1e-9) {
      stop_with_error("Invalid ee_link/tool_tcp TF");
      return false;
    }
    ee_tcp = KDL::Frame(KDL::Rotation::Quaternion(
        q.x/norm, q.y/norm, q.z/norm, q.w/norm), KDL::Vector(t.x, t.y, t.z));
    constexpr double pi = 3.14159265358979323846;
    const KDL::Frame nominal(
      KDL::Rotation::RotX(pi), KDL::Vector(0.036437755, 0.0, 0.115));
    const KDL::Twist error = KDL::diff(nominal, ee_tcp);
    if (error.vel.Norm() > 0.001 || error.rot.Norm() > 0.01) {
      stop_with_error("TCP TF differs from tested 5D NOMINAL model");
      return false;
    }
    return true;
  }

  JacobianMetrics jacobian_at(const KDL::JntArray &q, const KDL::Frame &ee_tcp,
    KDL::ChainFkSolverPos_recursive &fk, KDL::ChainJntToJacSolver &jac_solver) const
  {
    JacobianMetrics metrics;
    KDL::Frame ee;
    KDL::Jacobian jac(6);
    if (fk.JntToCart(q, ee) < 0 || jac_solver.JntToJac(q, jac) < 0) return metrics;
    // KDL Jacobian is computed at ee_link. Shift it to the real reference
    // point of the nominal TCP BEFORE calculating its singular values.
    jac.changeRefPoint(ee.M * ee_tcp.p);
    Eigen::Matrix<double, 6, 6> scaled = jac.data;
    // Linear and angular velocity rows have different units. Multiplying
    // angular rows by a stated characteristic length makes the diagnostic
    // condition number interpretable, but it remains scale-dependent.
    scaled.bottomRows<3>() *= characteristic_length_m_;
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(scaled);
    const auto s = svd.singularValues();
    metrics.valid = true;
    for (int i = 0; i < 6; ++i) metrics.singular[i] = s(i);
    metrics.sigma_max = s(0);
    metrics.sigma_min = s(5);
    if (s(5) > 1e-12) metrics.condition = s(0) / s(5);
    return metrics;
  }

  double joint_margin(const KDL::JntArray &q) const
  {
    double margin = std::numeric_limits<double>::infinity();
    for (unsigned int j = 0; j < 6; ++j) {
      margin = std::min(margin, std::min(q(j) - lower_(j), upper_(j) - q(j)));
    }
    return margin;
  }

  static std::string q_to_string(const KDL::JntArray &q)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << "[";
    for (unsigned int i = 0; i < q.rows(); ++i) {
      if (i) oss << ", ";
      oss << q(i);
    }
    oss << "]";
    return oss.str();
  }

  bool evaluate_solution(const KDL::JntArray &q, const KDL::Frame &desired_ee,
    KDL::ChainFkSolverPos_recursive &fk, double &pos_mm, double &rot_rad,
    double &margin) const
  {
    margin = joint_margin(q);
    for (unsigned int i = 0; i < q.rows(); ++i) {
      if (!std::isfinite(q(i)) || q(i) < lower_(i) || q(i) > upper_(i)) return false;
    }
    KDL::Frame result_ee;
    if (fk.JntToCart(q, result_ee) < 0) return false;
    const auto error = KDL::diff(desired_ee, result_ee);
    pos_mm = error.vel.Norm() * 1000.;
    rot_rad = error.rot.Norm();
    return std::isfinite(pos_mm) && std::isfinite(rot_rad) &&
           pos_mm <= 0.5 && rot_rad <= 0.002;
  }

  void log_retry(const char *label, int ret, const KDL::JntArray &q,
    const KDL::Frame &desired_ee, const KDL::JntArray &seed,
    KDL::ChainFkSolverPos_recursive &fk)
  {
    double pos_mm = std::numeric_limits<double>::quiet_NaN();
    double rot_rad = std::numeric_limits<double>::quiet_NaN();
    double margin = std::numeric_limits<double>::quiet_NaN();
    // A negative solver code is never accepted even if its output happens
    // to have a small FK residual. Still log the final iterate if finite.
    const bool pose_valid = evaluate_solution(
      q, desired_ee, fk, pos_mm, rot_rad, margin);
    const bool valid = ret >= 0 && pose_valid;
    double max_dq = 0.;
    for (unsigned int j = 0; j < 6; ++j) max_dq = std::max(max_dq, std::abs(q(j) - seed(j)));
    RCLCPP_WARN(get_logger(),
      "  RETRY %-17s code=%d valid=%s pos_err=%.6f mm rot_err=%.7f rad "
      "min_margin=%.5f rad max_dq=%.5f rad",
      label, ret, valid ? "YES" : "NO", pos_mm, rot_rad, margin, max_dq);
    // Even if an alternate IK succeeds, DO NOT publish or automatically
    // replace the original 5D trajectory. Step and velocity constraints
    // must still be checked separately.
  }

  void analyze_failure(const Stage &stage, int step, int ret,
    const KDL::JntArray &seed, const KDL::Frame &desired_ee,
    const KDL::Frame &desired_tcp, const KDL::Frame &ee_tcp,
    KDL::ChainFkSolverPos_recursive &fk, KDL::ChainJntToJacSolver &jac_solver)
  {
    KDL::Frame starting_ee;
    fk.JntToCart(seed, starting_ee);
    const KDL::Twist residual = KDL::diff(starting_ee, desired_ee);
    const auto metrics = jacobian_at(seed, ee_tcp, fk, jac_solver);
    RCLCPP_WARN(get_logger(),
      "  FAILURE: %s step %d/%d, baseline NR_JL(300) code=%d",
      stage.name, step, steps_, ret);
    RCLCPP_WARN(get_logger(),
      "  target TCP base_xyz=[%.6f, %.6f, %.6f] m",
      desired_tcp.p.x(), desired_tcp.p.y(), desired_tcp.p.z());
    RCLCPP_WARN(get_logger(), "  last valid joints rad=%s", q_to_string(seed).c_str());
    RCLCPP_WARN(get_logger(),
      "  target-vs-seed FK gap=%.6f mm / %.8f rad, "
      "joint limit margin=%.6f rad",
      residual.vel.Norm() * 1000., residual.rot.Norm(), joint_margin(seed));
    if (metrics.valid) {
      RCLCPP_WARN(get_logger(),
        "  scaled TCP Jacobian: sigma_min=%.9g sigma_max=%.9g condition=%.6g "
        "(angular rows scaled %.3f m)",
        metrics.sigma_min, metrics.sigma_max, metrics.condition, characteristic_length_m_);
      RCLCPP_WARN(get_logger(),
        "  sigmas(desc)=[%.7g, %.7g, %.7g, %.7g, %.7g, %.7g]",
        metrics.singular[0], metrics.singular[1], metrics.singular[2],
        metrics.singular[3], metrics.singular[4], metrics.singular[5]);
    }

    // Repeat from precisely the SAME last-valid seed and desired EE pose.
    // No retry result is used for execution or for the next diagnostic case.
    KDL::ChainIkSolverVel_pinv pinv(chain_);
    KDL::ChainIkSolverPos_NR_JL many_iter(
      chain_, lower_, upper_, fk, pinv, 1200, 1e-6);
    KDL::JntArray q_many = seed;
    const int ret_many = many_iter.CartToJnt(seed, desired_ee, q_many);
    log_retry("NR_JL(1200)", ret_many, q_many, desired_ee, seed, fk);

    KDL::ChainIkSolverVel_wdls wdls(chain_);
    wdls.setLambda(0.02);
    wdls.setEps(1e-5);
    KDL::ChainIkSolverPos_NR_JL damped(
      chain_, lower_, upper_, fk, wdls, 1200, 1e-6);
    KDL::JntArray q_damped = seed;
    const int ret_damped = damped.CartToJnt(seed, desired_ee, q_damped);
    log_retry("NR_JL+WDLS", ret_damped, q_damped, desired_ee, seed, fk);

    // LMA does not enforce joint limits: the validity gate checks limits
    // explicitly. A successful LMA result is diagnostic, not a safe path.
    KDL::ChainIkSolverPos_LMA lma(chain_, 1e-6, 500);
    KDL::JntArray q_lma = seed;
    const int ret_lma = lma.CartToJnt(seed, desired_ee, q_lma);
    log_retry("LMA(unbounded)", ret_lma, q_lma, desired_ee, seed, fk);
  }

  CaseResult diagnose_case(double transfer, const KDL::JntArray &start_q,
    const KDL::Frame &ee_tcp, KDL::ChainFkSolverPos_recursive &fk,
    KDL::ChainJntToJacSolver &jac_solver)
  {
    CaseResult result;
    KDL::ChainIkSolverVel_pinv ik_velocity(chain_);
    // Intentionally identical to 5D's baseline, 300 iterations, 1e-6.
    KDL::ChainIkSolverPos_NR_JL baseline(
      chain_, lower_, upper_, fk, ik_velocity, 300, 1e-6);
    const std::array<Stage, 6> stages = {{
      {"01_APPROACH_TCP_X", true,  KDL::Vector(approach_m_, 0., 0.)},
      {"02_INSERT_TCP_X", true,  KDL::Vector(insert_m_, 0., 0.)},
      {"03_LIFT_BASE_Z",   false, KDL::Vector(0., 0., lift_m_)},
      {"04_TRANSFER_BASE_X",false,KDL::Vector(transfer, 0., 0.)},
      {"05_LOWER_BASE_Z",  false, KDL::Vector(0., 0., -lower_m_)},
      {"06_WITHDRAW_TCP_X",true, KDL::Vector(-withdraw_m_, 0., 0.)}
    }};

    KDL::Frame start_ee;
    if (fk.JntToCart(start_q, start_ee) < 0) {
      result.reason = "Start FK failed";
      return result;
    }
    KDL::Frame stage_start = start_ee * ee_tcp;
    const KDL::Rotation orientation = stage_start.M;
    KDL::JntArray previous = start_q;
    const double dt = seconds_per_stage_ / static_cast<double>(steps_);

    for (const Stage &stage : stages) {
      const KDL::Vector delta_base = stage.tcp_local ?
        stage_start.M * stage.delta : stage.delta;
      for (int i = 1; i <= steps_; ++i) {
        const double s = static_cast<double>(i) / static_cast<double>(steps_);
        const KDL::Frame desired_tcp(orientation, stage_start.p + delta_base * s);
        const KDL::Frame desired_ee = desired_tcp * ee_tcp.Inverse();
        KDL::JntArray solved(6);
        const int ret = baseline.CartToJnt(previous, desired_ee, solved);
        if (ret < 0) {
          result.stage = stage.name;
          result.step = i;
          result.ik_code = ret;
          result.reason = "NR_JL(300) IK did not converge";
          analyze_failure(stage, i, ret, previous, desired_ee, desired_tcp,
            ee_tcp, fk, jac_solver);
          return result;
        }
        double pos_mm = 0., rot_rad = 0., margin = 0.;
        if (!evaluate_solution(solved, desired_ee, fk, pos_mm, rot_rad, margin)) {
          result.reason = "Solution outside limits or FK tolerance";
          result.stage = stage.name;
          result.step = i;
          return result;
        }
        result.max_fk_mm = std::max(result.max_fk_mm, pos_mm);
        result.min_joint_margin_rad = std::min(result.min_joint_margin_rad, margin);
        for (std::size_t j = 0; j < 6; ++j) {
          const double dq = std::abs(solved(j) - previous(j));
          const double ratio = dq / dt / velocity_limits_[j];
          result.max_joint_step = std::max(result.max_joint_step, dq);
          result.max_velocity_fraction = std::max(result.max_velocity_fraction, ratio);
          if (dq > max_joint_step_ || !std::isfinite(ratio) || ratio > 0.5) {
            result.stage = stage.name;
            result.step = i;
            result.reason = dq > max_joint_step_ ?
              "Joint step > configured limit" : "Velocity >50% URDF limit";
            return result;
          }
        }
        // Same discrete joint-space midpoint verification as 5D.
        KDL::JntArray mid(6);
        for (std::size_t j = 0; j < 6; ++j) mid(j) = (previous(j) + solved(j)) / 2.;
        KDL::Frame mid_ee;
        if (fk.JntToCart(mid, mid_ee) < 0) {
          result.stage = stage.name;
          result.step = i;
          result.reason = "Midpoint FK error";
          return result;
        }
        const KDL::Frame desired_mid(orientation,
          stage_start.p + delta_base * ((static_cast<double>(i) - 0.5) / steps_));
        const KDL::Twist mid_error = KDL::diff(desired_mid, mid_ee * ee_tcp);
        const double midpoint_mm = mid_error.vel.Norm() * 1000.;
        result.max_midpoint_mm = std::max(result.max_midpoint_mm, midpoint_mm);
        if (!std::isfinite(midpoint_mm) || midpoint_mm > 0.5 ||
            mid_error.rot.Norm() > 0.002) {
          result.stage = stage.name;
          result.step = i;
          result.reason = "Midpoint TCP tolerance exceeded";
          return result;
        }
        const auto metrics = jacobian_at(solved, ee_tcp, fk, jac_solver);
        if (metrics.valid) {
          result.smallest_sigma = std::min(result.smallest_sigma, metrics.sigma_min);
          result.largest_condition = std::max(result.largest_condition, metrics.condition);
        }
        previous = solved;
      }
      stage_start = KDL::Frame(orientation, stage_start.p + delta_base);
      RCLCPP_INFO(get_logger(), "  PASS %-21s  %2d waypoints", stage.name, steps_);
    }
    result.passed = true;
    result.reason = "All six stages pass 5D-equivalent gates";
    return result;
  }

  void run_diagnostics(const KDL::Frame &ee_tcp)
  {
    KDL::ChainFkSolverPos_recursive fk(chain_);
    KDL::ChainJntToJacSolver jac_solver(chain_);
    const KDL::JntArray immutable_start = snapshot_;
    KDL::Frame initial_ee;
    if (fk.JntToCart(immutable_start, initial_ee) < 0) {
      stop_with_error("Start FK failed");
      return;
    }
    const KDL::Frame initial_tcp = initial_ee * ee_tcp;
    const KDL::Vector insertion_base = initial_tcp.M * KDL::Vector(1., 0., 0.);
    RCLCPP_INFO(get_logger(), "========== 5E-1 IK DIAGNOSTIC START ==========");
    RCLCPP_INFO(get_logger(), "START SNAPSHOT joints rad=%s",
      q_to_string(immutable_start).c_str());
    for (std::size_t i = 0; i < 6; ++i) {
      RCLCPP_INFO(get_logger(), "  %-16s current=% .6f lower=% .6f upper=% .6f "
        "margin=% .6f rad", joint_names_[i].c_str(), immutable_start(i),
        lower_(i), upper_(i),
        std::min(immutable_start(i) - lower_(i), upper_(i) - immutable_start(i)));
    }
    RCLCPP_INFO(get_logger(),
      "START TCP base_xyz=[%.6f, %.6f, %.6f] m; TCP+X direction in base=[%.6f, %.6f, %.6f]",
      initial_tcp.p.x(), initial_tcp.p.y(), initial_tcp.p.z(),
      insertion_base.x(), insertion_base.y(), insertion_base.z());
    const auto jm = jacobian_at(immutable_start, ee_tcp, fk, jac_solver);
    if (jm.valid) {
      RCLCPP_INFO(get_logger(),
        "START TCP Jacobian sigma_min=%.8g condition=%.6g (rotation rows x %.2fm)",
        jm.sigma_min, jm.condition, characteristic_length_m_);
    }
    RCLCPP_WARN(get_logger(),
      "Each transfer case RESTARTS at exactly the same joint snapshot; no hardware motion.");
    RCLCPP_WARN(get_logger(),
      "NR_JL(300) matches 5D; extra solvers run ONLY at failure and never publish goals.");

    for (double transfer : transfer_distances_) {
      RCLCPP_INFO(get_logger(), "========== CASE transfer=%.3f mm ==========",
        transfer * 1000.);
      const CaseResult r = diagnose_case(transfer, immutable_start, ee_tcp, fk, jac_solver);
      if (r.passed) {
        RCLCPP_INFO(get_logger(),
          "CASE PASS transfer=%.3f mm; max_qstep=%.6f rad; max_vel=%.2f%% URDF; "
          "min_limit_margin=%.6f rad; min_sigma=%.9g; max_condition=%.6g; "
          "max_FK=%.6f mm; max_midpoint=%.6f mm",
          transfer * 1000., r.max_joint_step, r.max_velocity_fraction * 100.,
          r.min_joint_margin_rad, r.smallest_sigma, r.largest_condition,
          r.max_fk_mm, r.max_midpoint_mm);
      } else {
        RCLCPP_WARN(get_logger(),
          "CASE FAIL transfer=%.3f mm at %s step %d, code=%d, reason=%s",
          transfer * 1000., r.stage.c_str(), r.step, r.ik_code, r.reason.c_str());
      }
      outcomes_.push_back({transfer, r.passed});
    }
    std::ostringstream overview;
    overview << "5E-1 SUMMARY | ";
    for (const auto &outcome : outcomes_) {
      overview << std::fixed << std::setprecision(1) << outcome.first * 1000.
               << "mm:" << (outcome.second ? "PASS" : "FAIL") << " | ";
    }
    RCLCPP_INFO(get_logger(), "%s", overview.str().c_str());
    RCLCPP_WARN(get_logger(),
      "Diagnostic conclusion must distinguish KDL convergence, limits, and singularity. "
      "Alternative IK success does NOT imply a collision-safe/continuous trajectory.");
    RCLCPP_INFO(get_logger(), "5E-1 READ-ONLY DIAGNOSTIC COMPLETE (NO GOAL SENT)");
    finished_ = true;
    completed_ = true;
    rclcpp::shutdown();
  }

  void tick()
  {
    if (finished_) return;
    if (!params_ok()) return;
    if (!model_ready_ || !state_ready_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
        "Waiting for /robot_description and current /joint_states...");
    } else if (std::chrono::steady_clock::now() - last_state_ <= 1s) {
      KDL::Frame ee_tcp;
      if (read_nominal_tf(ee_tcp)) {
        run_diagnostics(ee_tcp);
        return;
      }
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
        "Waiting for fresh joint states...");
    }
    if (!finished_ && std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started_).count() > timeout_s_) {
      stop_with_error("Timed out waiting for URDF, fresh joint states, or TCP TF");
    }
  }

  KDL::Chain chain_;
  KDL::JntArray snapshot_, lower_, upper_;
  std::vector<std::string> joint_names_;
  std::vector<double> velocity_limits_;
  std::vector<double> transfer_distances_;
  std::vector<std::pair<double, bool>> outcomes_;
  double approach_m_{0.002}, insert_m_{0.005}, lift_m_{0.005};
  double lower_m_{0.005}, withdraw_m_{0.007};
  double seconds_per_stage_{2.0}, max_joint_step_{0.10};
  double characteristic_length_m_{0.30}, timeout_s_{20.};
  int steps_{10};
  bool acknowledged_{false}, model_ready_{false}, state_ready_{false};
  bool finished_{false}, completed_{false};
  std::chrono::steady_clock::time_point started_{}, last_state_{};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IkDiagnostic>();
  rclcpp::spin(node);
  const int return_code = node->completed() ? 0 : 1;
  if (rclcpp::ok()) rclcpp::shutdown();
  return return_code;
}
