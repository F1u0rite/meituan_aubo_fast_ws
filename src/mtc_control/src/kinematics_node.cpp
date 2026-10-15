#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <urdf/model.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;


// ============================================================
// AUBO S3 Kinematics Node
//
// 功能：
// 1. 从 /robot_description 读取 URDF
// 2. 建立 base_link -> ee_link KDL 运动学链
// 3. 从 /joint_states 获取当前关节角度
// 4. 从 URDF 提取关节上下限
// 5. 计算正运动学 FK
// 6. 与 ROS 2 TF 进行一致性验证
// 7. 验证当前位姿的逆运动学 IK
// 8. 验证非零目标位姿的逆运动学 IK
//
// 注意：
// - 本节点只读取数据、执行数学计算
// - 不发送任何机械臂运动指令
// - 不依赖 MoveIt
// ============================================================


class KinematicsNode : public rclcpp::Node
{
public:

    KinematicsNode()
        : Node("mtc_kinematics"),
          q_(6),
          q_min_(6),
          q_max_(6)
    {
        // ----------------------------------------------------
        // 1. 初始化 TF Buffer
        // ----------------------------------------------------

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(
                get_clock()
            );

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(
                *tf_buffer_
            );


        // ----------------------------------------------------
        // 2. 订阅机器人 URDF
        // ----------------------------------------------------

        urdf_sub_ =
            create_subscription<std_msgs::msg::String>(
                "/robot_description",
                rclcpp::QoS(1).reliable().transient_local(),

                [this](
                    std_msgs::msg::String::SharedPtr msg
                )
                {
                    on_urdf(msg);
                }
            );


        // ----------------------------------------------------
        // 3. 订阅机械臂关节状态
        // ----------------------------------------------------

        joint_sub_ =
            create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",
                rclcpp::SensorDataQoS(),

                [this](
                    sensor_msgs::msg::JointState::SharedPtr msg
                )
                {
                    on_joint_state(msg);
                }
            );


        // ----------------------------------------------------
        // 4. 定时执行测试
        // ----------------------------------------------------

        timer_ =
            create_wall_timer(
                500ms,
                [this]()
                {
                    run_test();
                }
            );


        RCLCPP_INFO(
            get_logger(),
            "========================================"
        );

        RCLCPP_INFO(
            get_logger(),
            "AUBO S3 Kinematics Node Started"
        );

        RCLCPP_INFO(
            get_logger(),
            "Mode: READ ONLY"
        );

        RCLCPP_INFO(
            get_logger(),
            "FK / TF / IK / Non-zero IK"
        );

        RCLCPP_INFO(
            get_logger(),
            "========================================"
        );
    }


private:

    // ========================================================
    // 1. URDF 初始化
    // ========================================================

    void on_urdf(
        const std_msgs::msg::String::SharedPtr msg)
    {
        if (model_ready_) {
            return;
        }

        urdf::Model robot;

        if (!robot.initString(msg->data)) {
            RCLCPP_ERROR(
                get_logger(),
                "URDF parsing failed"
            );
            return;
        }


        // ----------------------------------------------------
        // 将 URDF 转换为 KDL Tree
        // ----------------------------------------------------

        KDL::Tree tree;

        if (!kdl_parser::treeFromString(
                msg->data,
                tree)) {

            RCLCPP_ERROR(
                get_logger(),
                "KDL tree creation failed"
            );
            return;
        }


        // ----------------------------------------------------
        // 获取 base_link -> ee_link 运动学链
        // ----------------------------------------------------

        if (!tree.getChain(
                "base_link",
                "ee_link",
                chain_)) {

            RCLCPP_ERROR(
                get_logger(),
                "Cannot find base_link -> ee_link chain"
            );
            return;
        }


        if (chain_.getNrOfJoints() != 6) {

            RCLCPP_ERROR(
                get_logger(),
                "Expected 6 joints, got %u",
                chain_.getNrOfJoints()
            );
            return;
        }


        // ----------------------------------------------------
        // 从 URDF 提取关节名称和限位
        // ----------------------------------------------------

        joint_names_.clear();

        unsigned int index = 0;

        for (
            unsigned int i = 0;
            i < chain_.getNrOfSegments();
            ++i
        )
        {
            const auto &joint =
                chain_.getSegment(i).getJoint();

            if (joint.getType() == KDL::Joint::None) {
                continue;
            }

            const std::string name =
                joint.getName();

            const auto urdf_joint =
                robot.getJoint(name);

            if (!urdf_joint ||
                urdf_joint->type != urdf::Joint::REVOLUTE ||
                !urdf_joint->limits) {

                RCLCPP_ERROR(
                    get_logger(),
                    "Missing revolute joint limits: %s",
                    name.c_str()
                );

                return;
            }

            const double lower =
                urdf_joint->limits->lower;

            const double upper =
                urdf_joint->limits->upper;


            if (!std::isfinite(lower) ||
                !std::isfinite(upper) ||
                lower >= upper) {

                RCLCPP_ERROR(
                    get_logger(),
                    "Invalid limits for %s",
                    name.c_str()
                );

                return;
            }


            q_min_(index) = lower;
            q_max_(index) = upper;

            joint_names_.push_back(name);


            RCLCPP_INFO(
                get_logger(),
                "%s limits: [%.4f, %.4f] rad",
                name.c_str(),
                lower,
                upper
            );

            ++index;
        }


        if (index != 6) {

            RCLCPP_ERROR(
                get_logger(),
                "Joint extraction failed"
            );

            return;
        }


        model_ready_ = true;


        RCLCPP_INFO(
            get_logger(),
            "KDL chain ready: %u segments, %u joints",
            chain_.getNrOfSegments(),
            chain_.getNrOfJoints()
        );
    }



    // ========================================================
    // 2. 关节状态回调
    // ========================================================

    void on_joint_state(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!model_ready_) {
            return;
        }


        KDL::JntArray next_q(6);


        for (
            size_t i = 0;
            i < joint_names_.size();
            ++i
        )
        {
            const auto it =
                std::find(
                    msg->name.begin(),
                    msg->name.end(),
                    joint_names_[i]
                );


            if (it == msg->name.end()) {
                return;
            }


            const size_t index =
                std::distance(
                    msg->name.begin(),
                    it
                );


            if (index >= msg->position.size()) {
                return;
            }


            const double value =
                msg->position[index];


            if (!std::isfinite(value)) {
                return;
            }


            if (value < q_min_(i) ||
                value > q_max_(i)) {

                RCLCPP_ERROR_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Joint outside URDF limits: %s",
                    joint_names_[i].c_str()
                );

                return;
            }


            next_q(i) = value;
        }


        q_ = next_q;

        state_received_ = true;

        last_state_time_ =
            std::chrono::steady_clock::now();
    }



    // ========================================================
    // 3. 运行 FK / IK 测试
    // ========================================================

    void run_test()
    {
        if (finished_) {
            return;
        }


        // ----------------------------------------------------
        // 检查 URDF 和关节数据
        // ----------------------------------------------------

        if (!model_ready_ || !state_received_) {

            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Waiting for URDF and joint states..."
            );

            return;
        }


        // ----------------------------------------------------
        // 检查关节状态是否新鲜
        // ----------------------------------------------------

        if (
            std::chrono::steady_clock::now() -
            last_state_time_ > 1s
        )
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Joint state is stale"
            );

            return;
        }



        // ====================================================
        // TEST 1: 正运动学 FK
        // ====================================================

        RCLCPP_INFO(
            get_logger(),
            "---------- TEST 1: FK ----------"
        );


        KDL::ChainFkSolverPos_recursive
            fk_solver(chain_);


        KDL::Frame fk_pose;


        const int fk_ret =
            fk_solver.JntToCart(
                q_,
                fk_pose
            );


        if (fk_ret < 0) {

            RCLCPP_ERROR(
                get_logger(),
                "FK failed: %d",
                fk_ret
            );

            return;
        }



        RCLCPP_INFO(
            get_logger(),
            "FK XYZ = [%.6f, %.6f, %.6f] m",
            fk_pose.p.x(),
            fk_pose.p.y(),
            fk_pose.p.z()
        );



        // ====================================================
        // TEST 2: FK 与 ROS TF 对比
        // ====================================================

        RCLCPP_INFO(
            get_logger(),
            "---------- TEST 2: FK vs TF ----------"
        );


        geometry_msgs::msg::TransformStamped tf;


        try {

            tf = tf_buffer_->lookupTransform(
                "base_link",
                "ee_link",
                tf2::TimePointZero
            );

        }
        catch (
            const tf2::TransformException &ex
        )
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Waiting for TF: %s",
                ex.what()
            );

            return;
        }



        const auto &t =
            tf.transform.translation;

        const auto &r =
            tf.transform.rotation;



        KDL::Frame tf_pose(
            KDL::Rotation::Quaternion(
                r.x,
                r.y,
                r.z,
                r.w
            ),

            KDL::Vector(
                t.x,
                t.y,
                t.z
            )
        );



        const KDL::Twist tf_error =
            KDL::diff(
                fk_pose,
                tf_pose
            );



        const double position_error =
            tf_error.vel.Norm();


        const double rotation_error =
            tf_error.rot.Norm();



        RCLCPP_INFO(
            get_logger(),
            "TF XYZ = [%.6f, %.6f, %.6f] m",
            t.x,
            t.y,
            t.z
        );


        RCLCPP_INFO(
            get_logger(),
            "FK/TF position error = %.6f mm",
            position_error * 1000.0
        );


        RCLCPP_INFO(
            get_logger(),
            "FK/TF rotation error = %.6f rad",
            rotation_error
        );



        if (
            position_error > 0.005 ||
            rotation_error > 0.01
        )
        {
            RCLCPP_ERROR(
                get_logger(),
                "FK / TF consistency check FAILED"
            );

            finished_ = true;
            rclcpp::shutdown();

            return;
        }



        // ====================================================
        // TEST 3: 当前目标 IK
        // ====================================================

        RCLCPP_INFO(
            get_logger(),
            "---------- TEST 3: Current Pose IK ----------"
        );



        KDL::ChainIkSolverVel_pinv
            ik_vel(chain_);



        KDL::ChainIkSolverPos_NR_JL ik_solver(
            chain_,
            q_min_,
            q_max_,
            fk_solver,
            ik_vel,
            150,
            1e-6
        );



        KDL::JntArray q_result(6);



        const int ik_ret =
            ik_solver.CartToJnt(
                q_,
                fk_pose,
                q_result
            );



        if (ik_ret < 0) {

            RCLCPP_ERROR(
                get_logger(),
                "Current pose IK failed: %d",
                ik_ret
            );

            finished_ = true;
            rclcpp::shutdown();

            return;
        }



        KDL::Frame recovered_pose;



        if (
            fk_solver.JntToCart(
                q_result,
                recovered_pose
            ) < 0
        )
        {
            RCLCPP_ERROR(
                get_logger(),
                "IK result FK validation failed"
            );

            finished_ = true;
            rclcpp::shutdown();

            return;
        }



        const KDL::Twist ik_error =
            KDL::diff(
                fk_pose,
                recovered_pose
            );



        RCLCPP_INFO(
            get_logger(),
            "Current IK position error = %.6f mm",
            ik_error.vel.Norm() * 1000.0
        );


        RCLCPP_INFO(
            get_logger(),
            "Current IK rotation error = %.6f rad",
            ik_error.rot.Norm()
        );



        for (size_t i = 0; i < 6; ++i) {

            RCLCPP_INFO(
                get_logger(),
                "%s: current=%.6f IK=%.6f rad",
                joint_names_[i].c_str(),
                q_(i),
                q_result(i)
            );
        }



        // ====================================================
        // TEST 4: 非零目标逆运动学
        // ====================================================

        RCLCPP_INFO(
            get_logger(),
            "---------- TEST 4: Non-zero Target IK ----------"
        );



        // ----------------------------------------------------
        // 4.1 生成新的关节目标
        // ----------------------------------------------------

        KDL::JntArray q_target(6);


        for (size_t i = 0; i < 6; ++i) {

            q_target(i) = q_(i);
        }



        // J1 相对当前角度变化 0.02 rad
        // 此处只用于数学计算，不控制机械臂运动

        const double delta =
            (q_target(0) + 0.02 <= q_max_(0))
            ? 0.02
            : -0.02;


        q_target(0) += delta;


        RCLCPP_INFO(
            get_logger(),
            "Non-zero IK target J1 = %.6f rad",
            q_target(0)
        );



        // ----------------------------------------------------
        // 4.2 根据新关节角计算目标 TCP 位姿
        // ----------------------------------------------------

        KDL::Frame target_pose;


        if (
            fk_solver.JntToCart(
                q_target,
                target_pose
            ) < 0
        )
        {
            RCLCPP_ERROR(
                get_logger(),
                "Target FK failed"
            );

            finished_ = true;
            rclcpp::shutdown();

            return;
        }



        RCLCPP_INFO(
            get_logger(),
            "Target XYZ = [%.6f, %.6f, %.6f] m",
            target_pose.p.x(),
            target_pose.p.y(),
            target_pose.p.z()
        );



        // ----------------------------------------------------
        // 4.3 从原始关节角出发求解新目标
        // ----------------------------------------------------

        KDL::JntArray q_solved(6);



        const int solve_ret =
            ik_solver.CartToJnt(
                q_,
                target_pose,
                q_solved
            );



        if (solve_ret < 0) {

            RCLCPP_ERROR(
                get_logger(),
                "Non-zero target IK failed: %d",
                solve_ret
            );

            finished_ = true;
            rclcpp::shutdown();

            return;
        }



        // ----------------------------------------------------
        // 4.4 FK 复算逆解
        // ----------------------------------------------------

        KDL::Frame solved_pose;



        if (
            fk_solver.JntToCart(
                q_solved,
                solved_pose
            ) < 0
        )
        {
            RCLCPP_ERROR(
                get_logger(),
                "Solved FK failed"
            );

            finished_ = true;
            rclcpp::shutdown();

            return;
        }



        const KDL::Twist error =
            KDL::diff(
                target_pose,
                solved_pose
            );



        const double pos_error_mm =
            error.vel.Norm() * 1000.0;


        const double rot_error_rad =
            error.rot.Norm();



        RCLCPP_INFO(
            get_logger(),
            "NON-ZERO IK position error = %.6f mm",
            pos_error_mm
        );


        RCLCPP_INFO(
            get_logger(),
            "NON-ZERO IK rotation error = %.6f rad",
            rot_error_rad
        );



        // ----------------------------------------------------
        // 4.5 检查关节限位
        // ----------------------------------------------------

        for (size_t i = 0; i < 6; ++i) {

            if (
                !std::isfinite(q_solved(i)) ||
                q_solved(i) < q_min_(i) ||
                q_solved(i) > q_max_(i)
            )
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "IK result violates joint limits: %s",
                    joint_names_[i].c_str()
                );

                finished_ = true;
                rclcpp::shutdown();

                return;
            }



            RCLCPP_INFO(
                get_logger(),
                "%s: target=%.6f solved=%.6f rad",
                joint_names_[i].c_str(),
                q_target(i),
                q_solved(i)
            );
        }



        // ----------------------------------------------------
        // 4.6 检查运动学求解误差
        // ----------------------------------------------------

        if (
            pos_error_mm > 0.1 ||
            rot_error_rad > 0.001
        )
        {
            RCLCPP_ERROR(
                get_logger(),
                "Non-zero target IK accuracy check FAILED"
            );

            finished_ = true;
            rclcpp::shutdown();

            return;
        }



        RCLCPP_INFO(
            get_logger(),
            "SUCCESS: Non-zero target IK verified"
        );



        // ====================================================
        // ALL TESTS PASSED
        // ====================================================

        finished_ = true;


        RCLCPP_INFO(
            get_logger(),
            "========================================"
        );

        RCLCPP_INFO(
            get_logger(),
            "SUCCESS: ALL KINEMATICS TESTS PASSED"
        );

        RCLCPP_INFO(
            get_logger(),
            "1. FK: PASS"
        );

        RCLCPP_INFO(
            get_logger(),
            "2. FK / TF: PASS"
        );

        RCLCPP_INFO(
            get_logger(),
            "3. Current Pose IK: PASS"
        );

        RCLCPP_INFO(
            get_logger(),
            "4. Non-zero Target IK: PASS"
        );

        RCLCPP_INFO(
            get_logger(),
            "========================================"
        );


        rclcpp::shutdown();
    }



    // ========================================================
    // Member Variables
    // ========================================================

    KDL::Chain chain_;

    KDL::JntArray q_;
    KDL::JntArray q_min_;
    KDL::JntArray q_max_;


    std::vector<std::string> joint_names_;


    bool model_ready_{false};
    bool state_received_{false};
    bool finished_{false};


    std::chrono::steady_clock::time_point
        last_state_time_{};


    rclcpp::Subscription<
        std_msgs::msg::String
    >::SharedPtr urdf_sub_;


    rclcpp::Subscription<
        sensor_msgs::msg::JointState
    >::SharedPtr joint_sub_;


    rclcpp::TimerBase::SharedPtr timer_;


    std::unique_ptr<tf2_ros::Buffer>
        tf_buffer_;


    std::shared_ptr<tf2_ros::TransformListener>
        tf_listener_;
};



// ============================================================
// Main
// ============================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<KinematicsNode>();

    rclcpp::spin(node);

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    return 0;
}