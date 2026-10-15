// 5F-1: Meituan 2026 challenge BOARD-ILLUSTRATION coordinate model.
// No link to base_link or real-world calibration; no hardware or trajectory interaction.
// Source: official two-page "2026 challenge task base map" PDF.
// Dimensions in PDF text are official annotations; centre positions below are
// visual estimates of the drawing, NOT dimensioned or measured coordinates.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace {
constexpr double kBoardX = 0.700;
constexpr double kBoardY = 0.540;
constexpr double kBatteryX = 0.070;
constexpr double kBatteryY = 0.080;
constexpr double kBatteryZoneX = 0.300;
constexpr double kBatteryZoneY = 0.250;
constexpr double kT0X = 0.200;  // T0 is drawn with the long side left/right.
constexpr double kT0Y = 0.100;  // Annotated footprint: 100 x 200 mm.
constexpr double kRingOuterDiameter = 0.150;
constexpr double kRingInnerDiameter = 0.080;
constexpr double kBaseDiameter = 0.200;

struct XY { double x{}; double y{}; };
struct Color { float r{}, g{}, b{}, a{}; };
struct Footprint {
  std::string name;
  std::string frame;
  XY center;
  std::string shape; // rectangle, disk
  double x_size{};
  double y_size{};
  Color color;
};

XY read_xy(rclcpp::Node* node,
           const std::string& parameter_name, XY proposed) {
  const std::vector<double> default_pair{proposed.x, proposed.y};
  const auto values = node->declare_parameter<std::vector<double>>(parameter_name, default_pair);
  if (values.size() != 2 || !std::isfinite(values[0]) || !std::isfinite(values[1])) {
    throw std::runtime_error("Parameter " + parameter_name + " must be [finite_x,finite_y] in metres");
  }
  if (values[0] < 0 || values[0] > kBoardX || values[1] < 0 || values[1] > kBoardY) {
    throw std::runtime_error("Parameter " + parameter_name + " must have centre within board bounds");
  }
  return {values[0], values[1]};
}

std::string format_xyz(XY center) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(4) << '[' << center.x << ',' << center.y << ",0]";
  return s.str();
}

bool valid_frame_name(const std::string &name) {
  if (name.empty() || name.front() == '/') return false;
  for (char c : name) {
    const bool valid = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                       (c >= '0' && c <= '9') || c == '_' || c == '/';
    if (!valid) return false;
  }
  return true;
}
} // namespace

class TaskBoardNode final : public rclcpp::Node {
public:
  TaskBoardNode() : Node("mtc_task_board") {
    layout_ = this->declare_parameter<std::string>("layout", "basic");
    root_frame_ = this->declare_parameter<std::string>("root_frame", "task_board_illustration");
    publish_markers_ = this->declare_parameter<bool>("publish_markers", true);
    if (layout_ != "basic" && layout_ != "precision") {
      throw std::runtime_error("layout must be 'basic' (T0 page 1) or 'precision' (P1/P2/P3 page 2)");
    }
    if (!valid_frame_name(root_frame_)) {
      throw std::runtime_error("root_frame must be nonempty, without leading slash or spaces");
    }
    if (root_frame_ == "base_link" || root_frame_ == "world" || root_frame_ == "map") {
      throw std::runtime_error("5F-1 may not claim base_link/world/map as its unregistered illustration frame");
    }

    // All centres are approximate placements visually inferred from the PDF.
    // They are in a RIGHT-handed paper frame: +X right, +Y toward top of page,
    // +Z out of the paper; origin is at the bottom-left corner of the page.
    const XY battery_zone = read_xy(this,
      layout_ == "basic" ? "basic_battery_zone_xy_m" : "precision_battery_zone_xy_m",
      layout_ == "basic" ? XY{0.218, 0.415} : XY{0.180, 0.415});
    add("battery_zone", "battery_zone_planar_nominal", battery_zone, "rectangle",
        kBatteryZoneX, kBatteryZoneY, {0.55f, 0.58f, 0.62f, 0.19f});

    const double ax = layout_ == "basic" ? 0.178 : 0.140;
    const double bx = layout_ == "basic" ? 0.257 : 0.219;
    const std::string prefix = layout_ + "_";
    add_battery("A", read_xy(this, prefix + "battery_A_xy_m", {ax, 0.446}),
                {0.83f, 0.15f, 0.12f, 0.72f});
    add_battery("B", read_xy(this, prefix + "battery_B_xy_m", {bx, 0.446}),
                {0.97f, 0.78f, 0.08f, 0.72f});
    add_battery("C", read_xy(this, prefix + "battery_C_xy_m", {ax, 0.359}),
                {0.17f, 0.56f, 0.82f, 0.72f});
    add_battery("D", read_xy(this, prefix + "battery_D_xy_m", {bx, 0.359}),
                {0.38f, 0.70f, 0.16f, 0.72f});

    if (layout_ == "basic") {
      add("target_T0", "target_T0_planar_nominal",
          read_xy(this, "basic_target_T0_xy_m", {0.508, 0.400}),
          "rectangle", kT0X, kT0Y, {0.38f, 0.53f, 0.84f, 0.35f});
    } else {
      add_ring("P1", read_xy(this, "precision_target_P1_xy_m", {0.435, 0.464}));
      add_ring("P2", read_xy(this, "precision_target_P2_xy_m", {0.610, 0.464}));
      add_ring("P3", read_xy(this, "precision_target_P3_xy_m", {0.610, 0.289}));
    }

    add("base_reference", "base_reference_planar_nominal",
        read_xy(this, "base_reference_xy_m", {0.393, 0.100}),
        "disk", kBaseDiameter, kBaseDiameter, {0.34f, 0.35f, 0.37f, 0.24f});

    tf_pub_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    manifest_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/mtc/task_board_manifest", rclcpp::QoS(1).reliable().transient_local());
    if (publish_markers_) {
      markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/mtc/task_board_markers", rclcpp::QoS(1).reliable().transient_local());
    }

    publish_static_frames();
    manifest_ = make_manifest();
    // Keep alive to serve late joiners and refresh RViz on newly opened displays.
    timer_ = this->create_wall_timer(std::chrono::seconds(2), [this]() {
      std_msgs::msg::String msg;
      msg.data = manifest_;
      manifest_pub_->publish(msg);
      if (markers_pub_) markers_pub_->publish(make_markers());
    });
    {
      std_msgs::msg::String msg;
      msg.data = manifest_;
      manifest_pub_->publish(msg);
      if (markers_pub_) markers_pub_->publish(make_markers());
    }
    RCLCPP_WARN(get_logger(),
      "5F-1 ILLUSTRATION-ONLY: layout='%s', root='%s'; NO base_link registration, NO hook/ring Z, NO TRAJECTORY OR ROBOT MOTION",
      layout_.c_str(), root_frame_.c_str());
    RCLCPP_INFO(get_logger(),
      "5F-1 ready: %zu nominal footprints; /mtc/task_board_manifest + %s + /tf_static",
      footprints_.size(), publish_markers_ ? "/mtc/task_board_markers" : "markers disabled");
  }

private:
  void add(std::string name, std::string frame, XY centre, std::string shape,
           double sx, double sy, Color col) {
    if (sx <= 0 || sy <= 0) throw std::runtime_error("Invalid footprint size");
    if (!valid_frame_name(frame) || frame == root_frame_) {
      throw std::runtime_error("Invalid/duplicate root child frame name: " + frame);
    }
    footprints_.push_back({std::move(name), std::move(frame), centre,
                           std::move(shape), sx, sy, col});
  }

  void add_battery(const std::string &id, XY center, Color color) {
    add("battery_" + id, "battery_" + id + "_planar_nominal", center,
        "rectangle", kBatteryX, kBatteryY, color);
  }
  void add_ring(const std::string &id, XY center) {
    add("target_" + id + "_outer", "target_" + id + "_planar_nominal", center,
        "disk", kRingOuterDiameter, kRingOuterDiameter, {0.65f, 0.50f, 0.84f, 0.24f});
    // Inner ring is a footprint layer, NOT an additional TF at identical xyz.
    add("target_" + id + "_inner", "", center, "disk",
        kRingInnerDiameter, kRingInnerDiameter, {0.42f, 0.80f, 0.72f, 0.25f});
  }

  void publish_static_frames() {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for (const auto &item : footprints_) {
      if (item.frame.empty()) continue;
      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = root_frame_;
      msg.child_frame_id = item.frame;
      msg.transform.translation.x = item.center.x;
      msg.transform.translation.y = item.center.y;
      msg.transform.translation.z = 0.;  // strictly paper surface, NOT ring/battery Z
      msg.transform.rotation.w = 1.;
      transforms.push_back(std::move(msg));
    }
    tf_pub_->sendTransform(transforms);
  }

  std::string make_manifest() const {
    std::ostringstream s;
    s << std::fixed << std::setprecision(4);
    s << "5F-1 DRAWING-ONLY / ESTIMATED CENTRES / NO BASE REGISTRATION / NO MOTION\n";
    s << "source=official 2026 challenge base map PDF, page="
      << (layout_ == "basic" ? "1" : "2") << "\n";
    s << "layout=" << layout_ << " root=" << root_frame_ << "\n";
    s << "units=m; origin=PAGE_BOTTOM_LEFT; +x=RIGHT; +y=UP; +z=OUT_OF_PAPER\n";
    s << "paper=[0.7000,0.5400]m (source annotated); battery=[0.0700,0.0800]m\n";
    s << "battery_zone=[0.3000,0.2500]m (paper rectangle orientation)\n";
    if (layout_ == "basic") {
      s << "T0=[0.2000,0.1000]m (horizontal long side, drawing annotation 100x200 mm)\n";
    } else {
      s << "P_outer_diameter=0.1500m P_inner_diameter=0.0800m (circular drawing)\n";
    }
    s << "base_reference_diameter=0.2000m (drawn ONLY: NOT base_link)\n";
    for (const auto& f : footprints_) {
      s << f.name << " centre=" << format_xyz(f.center)
        << " shape=" << f.shape << " size_xy=[" << f.x_size << ',' << f.y_size
        << "]m frame=" << (f.frame.empty() ? "none (visual layer)" : f.frame) << "\n";
    }
    s << "CRITICAL: centre coordinates estimated visually from un-dimensioned artwork;"
         " not surveyed/calibrated placements. Z=0 is the BOARD PLANE ONLY.\n";
    s << "CRITICAL: root has no TF relation to base_link or actual board;"
         " frame data MUST NOT be used for any arm IK / execution.\n";
    return s.str();
  }

  visualization_msgs::msg::Marker make_shape(int id, const Footprint &f, double z) const {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = root_frame_;
    m.header.stamp = now();
    m.ns = "mtc_5f1_illustration";
    m.id = id;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = f.shape == "disk" ? visualization_msgs::msg::Marker::CYLINDER :
                                 visualization_msgs::msg::Marker::CUBE;
    m.pose.orientation.w = 1.;
    m.pose.position.x = f.center.x;
    m.pose.position.y = f.center.y;
    m.pose.position.z = z;
    m.scale.x = f.x_size;
    m.scale.y = f.y_size;
    m.scale.z = 0.0008;
    m.color.r = f.color.r;
    m.color.g = f.color.g;
    m.color.b = f.color.b;
    m.color.a = f.color.a;
    m.lifetime = rclcpp::Duration::from_seconds(0);
    return m;
  }

  visualization_msgs::msg::Marker make_label(int id, const Footprint &f) const {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = root_frame_;
    m.header.stamp = now();
    m.ns = "mtc_5f1_labels";
    m.id = id;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.pose.orientation.w = 1.;
    m.pose.position.x = f.center.x;
    m.pose.position.y = f.center.y;
    m.pose.position.z = 0.008;
    m.scale.z = 0.019;
    m.text = f.name;
    m.color.r = 0.09f;
    m.color.g = 0.10f;
    m.color.b = 0.14f;
    m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(0);
    return m;
  }

  visualization_msgs::msg::MarkerArray make_markers() const {
    visualization_msgs::msg::MarkerArray array;
    Footprint paper{"paper", "", {kBoardX / 2., kBoardY / 2.},
                    "rectangle", kBoardX, kBoardY, {0.80f, 0.83f, 0.87f, 0.13f}};
    array.markers.push_back(make_shape(0, paper, -0.0010));
    for (size_t i = 0; i < footprints_.size(); ++i) {
      const auto &f = footprints_[i];
      // Layer order is just for visual legibility; no 3-D object volumes.
      array.markers.push_back(make_shape(static_cast<int>(i) + 1, f,
                                        0.0002 + static_cast<double>(i) * 0.00002));
      if (f.name.find("_inner") == std::string::npos && f.name != "battery_zone") {
        array.markers.push_back(make_label(static_cast<int>(i) + 100, f));
      }
    }
    return array;
  }

  std::string layout_, root_frame_, manifest_;
  bool publish_markers_ = true;
  std::vector<Footprint> footprints_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manifest_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TaskBoardNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("mtc_task_board"), "5F-1 setup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
