#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// ============================================================
// V1 Tool Frames
//
// ee_link
//   |
//   |  Nominal transform (optional, requires calibration)
//   v
// tool_mount_cad
//   |
//   +---- tool_tcp
//   |
//   +---- hook_entry
//
// Geometry extracted from the finalized V1 STEP.
//
// READ ONLY:
//   This node only publishes TF.
//   It never sends robot motion commands.
// ============================================================

class ToolFramesNode : public rclcpp::Node
{
public:
    ToolFramesNode()
        : Node("mtc_tool_frames")
    {
        // ----------------------------------------------------
        // Parameters
        // ----------------------------------------------------

        publish_nominal_mount_ =
            declare_parameter<bool>(
                "publish_nominal_mount",
                false);

        mount_yaw_deg_ =
            declare_parameter<double>(
                "mount_yaw_deg",
                0.0);

        ee_mount_xyz_ =
            declare_parameter<std::vector<double>>(
                "ee_mount_xyz",
                {0.0, 0.0, 0.0});

        if (ee_mount_xyz_.size() != 3 ||
            !std::isfinite(mount_yaw_deg_)) {
            throw std::runtime_error(
                "Invalid mounting parameters");
        }

        for (double value : ee_mount_xyz_) {
            if (!std::isfinite(value)) {
                throw std::runtime_error(
                    "Non-finite mounting translation");
            }
        }

        broadcaster_ =
            std::make_shared<
                tf2_ros::StaticTransformBroadcaster>(this);

        publish_frames();
    }

private:

    geometry_msgs::msg::TransformStamped make_tf(
        const std::string &parent,
        const std::string &child,
        double x,
        double y,
        double z,
        const tf2::Quaternion &q)
    {
        geometry_msgs::msg::TransformStamped tf;

        tf.header.stamp = now();

        tf.header.frame_id = parent;
        tf.child_frame_id = child;

        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = z;

        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        return tf;
    }

    void publish_frames()
    {
        std::vector<
            geometry_msgs::msg::TransformStamped
        > transforms;

        // ====================================================
        // 1. CAD Coordinate Convention
        // ====================================================

        // CAD mounting plane center:
        // (0, -7.5, 0) mm
        //
        // tool_mount_cad axes are parallel to CAD XYZ.
        //
        // Tool TCP axes:
        //   TCP +X = CAD +Z  (insertion direction)
        //   TCP +Y = CAD -X
        //   TCP +Z = CAD -Y  (support face normal)

        tf2::Quaternion q_tcp(
            0.5,
           -0.5,
            0.5,
            0.5);

        q_tcp.normalize();

        // ====================================================
        // 2. Support TCP
        // ====================================================

        // STEP face centroid:
        // CAD (0, 107.5, 36.437755) mm
        //
        // Relative to CAD mounting plane Y=-7.5:
        // (0, 115, 36.437755) mm

        transforms.push_back(
            make_tf(
                "tool_mount_cad",
                "tool_tcp",

                0.0,
                0.115,
                0.036437755,

                q_tcp));

        // ====================================================
        // 3. Entry Reference
        // ====================================================

        // Nominal leading-edge reference:
        // CAD (0, 107.5, 75) mm
        //
        // Relative to mounting reference:
        // (0, 115, 75) mm

        transforms.push_back(
            make_tf(
                "tool_mount_cad",
                "hook_entry",

                0.0,
                0.115,
                0.075,

                q_tcp));

        // ====================================================
        // 4. Optional EE -> CAD Mount Transform
        // ====================================================

        if (publish_nominal_mount_) {

            // NOMINAL ONLY.
            //
            // Assumptions:
            //   EE +Z follows tool shaft outward.
            //   Mounting reference is at EE origin.
            //   Mounting yaw is supplied as a parameter.
            //
            // CAD +X -> EE +Y
            // CAD +Y -> EE +Z
            // CAD +Z -> EE +X

            tf2::Quaternion q_nominal(
                0.5,
                0.5,
                0.5,
                0.5);

            const double pi =
                3.14159265358979323846;

            const double yaw =
                mount_yaw_deg_ * pi / 180.0;

            tf2::Quaternion q_yaw;

            q_yaw.setRPY(0.0, 0.0, yaw);

            tf2::Quaternion q_mount =
                q_yaw * q_nominal;

            q_mount.normalize();

            transforms.push_back(
                make_tf(
                    "ee_link",
                    "tool_mount_cad",

                    ee_mount_xyz_[0],
                    ee_mount_xyz_[1],
                    ee_mount_xyz_[2],

                    q_mount));

            RCLCPP_WARN(
                get_logger(),
                "Publishing NOMINAL ee_link -> "
                "tool_mount_cad transform");

            RCLCPP_WARN(
                get_logger(),
                "This is NOT a calibrated "
                "robot-tool transform");

            RCLCPP_INFO(
                get_logger(),
                "Nominal mount yaw = %.3f deg",
                mount_yaw_deg_);
        }

        broadcaster_->sendTransform(transforms);

        RCLCPP_INFO(
            get_logger(),
            "========================================");

        RCLCPP_INFO(
            get_logger(),
            "V1 Tool Frames Published");

        RCLCPP_INFO(
            get_logger(),
            "tool_mount_cad -> tool_tcp");

        RCLCPP_INFO(
            get_logger(),
            "tool_mount_cad -> hook_entry");

        RCLCPP_INFO(
            get_logger(),
            "TCP translation: "
            "[0, 115, 36.437755] mm");

        RCLCPP_INFO(
            get_logger(),
            "Entry translation: "
            "[0, 115, 75] mm");

        RCLCPP_INFO(
            get_logger(),
            "Nominal EE attachment: %s",
            publish_nominal_mount_
                ? "ENABLED"
                : "DISABLED");

        RCLCPP_INFO(
            get_logger(),
            "READ ONLY: No motion commands");

        RCLCPP_INFO(
            get_logger(),
            "========================================");
    }

    bool publish_nominal_mount_{false};

    double mount_yaw_deg_{0.0};

    std::vector<double> ee_mount_xyz_;

    std::shared_ptr<
        tf2_ros::StaticTransformBroadcaster
    > broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node =
            std::make_shared<ToolFramesNode>();

        rclcpp::spin(node);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(
            rclcpp::get_logger("tool_frames_main"),
            "Failed: %s",
            e.what());

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
