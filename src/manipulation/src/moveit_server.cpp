#include <tf2/LinearMath/Matrix3x3.h>

#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "helper/srv/move_request.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MoveitServer {
   public:
    MoveitServer(const rclcpp::Node::SharedPtr& node) : node_(node) {
        using namespace std::placeholders;

        RCLCPP_INFO(node_->get_logger(), "Starting MoveIt Server...");

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, "ur_manipulator", std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration::from_seconds(5.0));

        // Configure planner parameters
        node_->declare_parameter("planning_time", 10.0);
        node_->declare_parameter("goal_joint_tolerance", 0.001);
        node_->declare_parameter("goal_position_tolerance", 0.001);
        node_->declare_parameter("goal_orientation_tolerance", 0.001);

        setupCollisionObjects();

        // Apply parameters
        move_group_->setPlanningTime(node_->get_parameter("planning_time").as_double());
        move_group_->setGoalJointTolerance(node_->get_parameter("goal_joint_tolerance").as_double());
        move_group_->setGoalPositionTolerance(node_->get_parameter("goal_position_tolerance").as_double());
        move_group_->setGoalOrientationTolerance(node_->get_parameter("goal_orientation_tolerance").as_double());
        // move_group_->setPlannerId("RRTConnectkConfigDefault");
        // move_group_->setPlannerId("BKPIECEkConfigDefault");
        move_group_->setPlannerId("TRRTkConfigDefault");

        service_ =
            node_->create_service<helper::srv::MoveRequest>("/moveit_path_plan", std::bind(&MoveitServer::handle_request, this, _1, _2));
    }

    double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

    moveit_msgs::msg::Constraints set_constraint(const std::string& constraint_str) {
        moveit_msgs::msg::Constraints constraints;

        // Orientation Constraints
        if (str_contains(constraint_str, ORIEN)) {
            moveit_msgs::msg::OrientationConstraint orien;

            orien.link_name = move_group_->getEndEffectorLink();
            orien.header.frame_id = move_group_->getPlanningFrame();
            orien.absolute_x_axis_tolerance = 0.1;
            orien.absolute_y_axis_tolerance = 0.1;
            orien.absolute_z_axis_tolerance = 1.57;
            orien.parameterization = moveit_msgs::msg::OrientationConstraint::XYZ_EULER_ANGLES;
            orien.weight = 1.0;

            geometry_msgs::msg::Quaternion q;
            q.x = -0.70654;
            q.y = 0.70767;
            q.z = 0.0;
            q.w = 0.0012;
            orien.orientation = q;
            constraints.orientation_constraints.push_back(orien);
        }

        // Joint Constraints
        if (str_contains(constraint_str, WRIST)) {
            // Wrist 1: around -105°
            {
                moveit_msgs::msg::JointConstraint wrist1;
                wrist1.joint_name = "wrist_1_joint";
                wrist1.position = -105.0 * M_PI / 180.0;
                wrist1.tolerance_below = 0.1;
                wrist1.tolerance_above = 0.1;
                wrist1.weight = 1.0;
                constraints.joint_constraints.push_back(wrist1);
            }

            // Wrist 2: around -90°
            {
                moveit_msgs::msg::JointConstraint wrist2;
                wrist2.joint_name = "wrist_2_joint";
                wrist2.position = -90.0 * M_PI / 180.0;
                wrist2.tolerance_below = 0.1;
                wrist2.tolerance_above = 0.1;
                wrist2.weight = 1.0;
                constraints.joint_constraints.push_back(wrist2);
            }

            // Wrist 3: around 0°
            {
                moveit_msgs::msg::JointConstraint wrist3;
                wrist3.joint_name = "wrist_3_joint";
                wrist3.position = 0.0;
                wrist3.tolerance_below = 0.1;
                wrist3.tolerance_above = 0.1;
                wrist3.weight = 1.0;
                constraints.joint_constraints.push_back(wrist3);
            }

            RCLCPP_INFO(node_->get_logger(), "Wrist joint constraints applied.");
        }

        return constraints;
    }

    void handle_request(const std::shared_ptr<helper::srv::MoveRequest::Request> request,
                        std::shared_ptr<helper::srv::MoveRequest::Response> response) {
        RCLCPP_INFO(node_->get_logger(), "Received MoveIt path planning request. Command: %s", request->command.c_str());

        if (request->positions.size() != 6) {
            RCLCPP_ERROR(node_->get_logger(), "Expected 6 position elements, got %zu", request->positions.size());
            response->success = false;
            return;
        }

        // Clear previous targets and constraints
        move_group_->clearPoseTargets();
        move_group_->clearPathConstraints();

        // Handle different command types
        bool target_set = false;
        if (request->command == "cartesian") {
            target_set = set_cartesian_target(request->positions);
        } else if (request->command == "joint") {
            target_set = set_joint_target(request->positions);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Invalid command: %s", request->command.c_str());
            response->success = false;
            return;
        }

        if (!target_set) {
            response->success = false;
            return;
        }

        // Apply constraints if specified
        if (request->constraints_identifier != NONE) {
            move_group_->setPathConstraints(set_constraint(request->constraints_identifier));
        }

        // Common planning and execution logic
        response->success = plan_and_execute();
    }

    void setupCollisionObjects() {
        std::string frame_id = "world";
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 0.02, 0.9, 0.85, -0.18, 0.435, frame_id, "backWall"));
        planning_scene_interface.applyCollisionObject(generateCollisionObject(0.02, 1.5, 0.9, -0.25, 0.57, 0.435, frame_id, "sideWall"));
        planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 1.5, 0.02, 0.85, 0.57, -0.01, frame_id, "table"));
        planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 1.5, 0.02, 0.85, 0.57, 0.87, frame_id, "ceiling"));
    }

    auto generateCollisionObject(float sx, float sy, float sz, float x, float y, float z, const std::string& frame_id,
                                 const std::string& id) -> moveit_msgs::msg::CollisionObject {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = id;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {sx, sy, sz};

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = x;
        box_pose.position.y = y;
        box_pose.position.z = z;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }

    bool str_contains(const std::string& str, const std::string& substr) {
        if (substr.empty()) return false;
        return str.find(substr) != std::string::npos;
    }

   private:
    bool set_cartesian_target(const std::vector<double>& positions) {
        try {
            geometry_msgs::msg::Pose target_pose = create_pose_from_positions(positions);
            move_group_->setPoseTarget(target_pose);
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set Cartesian target: %s", e.what());
            return false;
        }
    }

    bool set_joint_target(const std::vector<double>& positions) {
        try {
            auto joint_targets = create_joint_map_from_positions(positions);
            move_group_->setJointValueTarget(joint_targets);
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set joint target: %s", e.what());
            return false;
        }
    }

    geometry_msgs::msg::Pose create_pose_from_positions(const std::vector<double>& positions) {
        tf2::Quaternion q;
        q.setRPY(deg2rad(positions[3]), deg2rad(positions[4]), deg2rad(positions[5]));
        q.normalize();

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = positions[0];
        target_pose.position.y = positions[1];
        target_pose.position.z = positions[2];
        target_pose.orientation = tf2::toMsg(q);

        return target_pose;
    }

    std::map<std::string, double> create_joint_map_from_positions(const std::vector<double>& positions) {
        return {{"shoulder_pan_joint", deg2rad(positions[0])}, {"shoulder_lift_joint", deg2rad(positions[1])},
                {"elbow_joint", deg2rad(positions[2])},        {"wrist_1_joint", deg2rad(positions[3])},
                {"wrist_2_joint", deg2rad(positions[4])},      {"wrist_3_joint", deg2rad(positions[5])}};
    }

    bool plan_and_execute() {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = false;
        int attempts = 0;
        const int max_attempts = 100;

        while (!success && attempts < max_attempts) {
            success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            attempts++;
            if (!success) {
                RCLCPP_WARN(node_->get_logger(), "Planning attempt %d failed, retrying...", attempts);
                move_group_->setPlanningTime(move_group_->getPlanningTime() + 2.0);
            }
        }

        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Plan successful after %d attempts. Executing...", attempts);
            move_group_->execute(plan);
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed after %d attempts.", max_attempts);
            return false;
        }
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Service<helper::srv::MoveRequest>::SharedPtr service_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;

    const std::string NONE = "NONE";
    const std::string ORIEN = "ORIEN";
    const std::string WRIST = "WRIST";
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_server");
    MoveitServer server(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// Home pose Joint
// ros2 service call /moveit_path_plan helper/srv/MoveRequest "{command: 'joint', positions: [0,
// -74.5, 90, -105, -90, 0]}"

// Home pose Cartesian
// ros2 service call /moveit_path_plan helper/srv/MoveRequest "{command: 'cartesian', positions:
// [0.59, 0.133, 0.366, 180, 0, 90]}"