#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.h"
#include "helper/action/draw_shape.hpp"
#include "helper/action/erase_grid.hpp"
#include "helper/srv/move_request.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MoveitServer {
   public:
    using DrawShape = helper::action::DrawShape;
    using EraseGrid = helper::action::EraseGrid;
    using GoalHandleDrawShape = rclcpp_action::ServerGoalHandle<DrawShape>;
    using GoalHandleEraseGrid = rclcpp_action::ServerGoalHandle<EraseGrid>;

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

        // Drawing parameters
        node_->declare_parameter("cell_size", 0.05);
        node_->declare_parameter("drawing_height", 0.17);
        node_->declare_parameter("lift_height", node_->get_parameter("drawing_height").as_double() + 0.05);
        node_->declare_parameter("x_offset", -0.065);
        node_->declare_parameter("y_offset", 0.020);
        node_->declare_parameter("erase_offset", 0.010);
        node_->declare_parameter("home_joint_positions", std::vector<double>{0.0, -103.5, 106.1, -92.6, -90.0, 0.0});

        setupCollisionObjects();

        // Apply parameters
        move_group_->setPlanningTime(node_->get_parameter("planning_time").as_double());
        move_group_->setGoalJointTolerance(node_->get_parameter("goal_joint_tolerance").as_double());
        move_group_->setGoalPositionTolerance(node_->get_parameter("goal_position_tolerance").as_double());
        move_group_->setGoalOrientationTolerance(node_->get_parameter("goal_orientation_tolerance").as_double());
        // move_group_->setPlannerId("RRTConnectkConfigDefault");
        // move_group_->setPlannerId("BKPIECEkConfigDefault");
        move_group_->setPlannerId("TRRTkConfigDefault");
        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);

        service_ =
            node_->create_service<helper::srv::MoveRequest>("/moveit_path_plan", std::bind(&MoveitServer::handle_request, this, _1, _2));

        draw_action_server_ = rclcpp_action::create_server<DrawShape>(
            node_, "manipulation/draw_shape", std::bind(&MoveitServer::handle_draw_goal, this, _1, _2),
            std::bind(&MoveitServer::handle_draw_cancel, this, _1), std::bind(&MoveitServer::handle_draw_accepted, this, _1));

        erase_action_server_ = rclcpp_action::create_server<EraseGrid>(
            node_, "manipulation/erase_grid", std::bind(&MoveitServer::handle_erase_goal, this, _1, _2),
            std::bind(&MoveitServer::handle_erase_cancel, this, _1), std::bind(&MoveitServer::handle_erase_accepted, this, _1));
        RCLCPP_INFO(node_->get_logger(), "MoveIt Server is ready.");
        return_home();
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

            RCLCPP_INFO(node_->get_logger(), "Orientation constraint applied.");
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

    // == Action Server Stuff ==
    rclcpp_action::GoalResponse handle_draw_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DrawShape::Goal> goal) {
        RCLCPP_INFO(node_->get_logger(), "Received goal request to draw '%s'", goal->shape.c_str());
        (void)uuid;

        if (goal->shape != "X" && goal->shape != "O") {
            RCLCPP_ERROR(node_->get_logger(), "Invalid shape: %s", goal->shape.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_draw_cancel(const std::shared_ptr<GoalHandleDrawShape> goal_handle) {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_draw_accepted(const std::shared_ptr<GoalHandleDrawShape> goal_handle) {
        std::thread{std::bind(&MoveitServer::execute_draw_shape, this, goal_handle)}.detach();
    }

    // ---------------------------------------------
    rclcpp_action::GoalResponse handle_erase_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const EraseGrid::Goal> goal) {
        RCLCPP_INFO(node_->get_logger(), "Received erase goal for %zu cells", goal->cell_poses.size());
        (void)uuid;

        if (goal->cell_poses.size() != 6) {
            RCLCPP_ERROR(node_->get_logger(), "Expected 6 cell poses for erasing, got %zu", goal->cell_poses.size());
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_erase_cancel(const std::shared_ptr<GoalHandleEraseGrid> goal_handle) {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel erase goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_erase_accepted(const std::shared_ptr<GoalHandleEraseGrid> goal_handle) {
        std::thread{std::bind(&MoveitServer::execute_erase_grid, this, goal_handle)}.detach();
    }

    // == Drawing Functions ==
    void execute_draw_shape(const std::shared_ptr<GoalHandleDrawShape> goal_handle) {
        RCLCPP_INFO(node_->get_logger(), "Executing draw shape goal...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<DrawShape::Feedback>();
        auto result = std::make_shared<DrawShape::Result>();

        // Extract grid cell center position
        double cell_x = goal->pose.x;
        double cell_y = goal->pose.y;
        double grid_theta = goal->pose.theta;  // Grid orientation

        // Apply constraints
        move_group_->clearPathConstraints();
        if (goal->constraints_identifier != NONE) {
            move_group_->setPathConstraints(set_constraint(goal->constraints_identifier));
        }

        bool success = false;

        if (goal->shape == "X") {
            success = draw_x(cell_x, cell_y, grid_theta, goal_handle, feedback);
        } else if (goal->shape == "O") {
            success = draw_o(cell_x, cell_y, goal_handle, feedback);
        }

        if (success) {
            // Return home
            feedback->status = "returning_home";
            feedback->progress = 0.95;
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            if (return_home()) {
                result->success = true;
                result->message = "Shape drawn successfully";
                goal_handle->succeed(result);
                RCLCPP_INFO(node_->get_logger(), "Goal succeeded!");
            } else {
                result->success = false;
                result->message = "Failed to return home";
                goal_handle->abort(result);
            }
        } else {
            result->success = false;
            result->message = "Failed to draw shape";
            goal_handle->abort(result);
        }

        // Clear constraints after drawing
        move_group_->clearPathConstraints();
    }

    bool draw_x(double cx, double cy, double theta, const std::shared_ptr<GoalHandleDrawShape>& goal_handle,
                std::shared_ptr<DrawShape::Feedback>& feedback) {
        RCLCPP_INFO(node_->get_logger(), "Drawing X at (%.3f, %.3f) with theta=%.3f", cx, cy, theta);

        double cell_size = node_->get_parameter("cell_size").as_double();
        double drawing_height = node_->get_parameter("drawing_height").as_double();
        double lift_height = node_->get_parameter("lift_height").as_double();
        double x_offset = node_->get_parameter("x_offset").as_double();
        double y_offset = node_->get_parameter("y_offset").as_double();
        double half = cell_size / 2.0;
        cx += x_offset;
        cy += y_offset;

        // Calculate corners accounting for grid rotation
        double cos_t = std::cos(theta);
        double sin_t = std::sin(theta);

        // Top-left corner
        double tl_x = cx + (-half * cos_t - (-half) * sin_t);
        double tl_y = cy + (-half * sin_t + (-half) * cos_t);

        // Bottom-right corner
        double br_x = cx + (half * cos_t - half * sin_t);
        double br_y = cy + (half * sin_t + half * cos_t);

        // Top-right corner
        double tr_x = cx + (half * cos_t - (-half) * sin_t);
        double tr_y = cy + (half * sin_t + (-half) * cos_t);

        // Bottom-left corner
        double bl_x = cx + ((-half) * cos_t - half * sin_t);
        double bl_y = cy + ((-half) * sin_t + half * cos_t);

        std::vector<geometry_msgs::msg::Pose> waypoints;

        // First diagonal: top-left to bottom-right
        feedback->status = "moving_to_start";
        feedback->progress = 0.1;
        goal_handle->publish_feedback(feedback);

        // Move above top-left (lifted)
        waypoints.push_back(create_pose(tl_x, tl_y, lift_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // Descend to drawing height
        feedback->status = "descending";
        feedback->progress = 0.2;
        goal_handle->publish_feedback(feedback);

        waypoints.push_back(create_pose(tl_x, tl_y, drawing_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // Draw line to bottom-right
        feedback->status = "drawing_line_1";
        feedback->progress = 0.35;
        goal_handle->publish_feedback(feedback);

        waypoints.push_back(create_pose(br_x, br_y, drawing_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // Lift up
        feedback->status = "lifting";
        feedback->progress = 0.5;
        goal_handle->publish_feedback(feedback);

        waypoints.push_back(create_pose(br_x, br_y, lift_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // Second diagonal: move to top-right
        feedback->status = "moving_to_second_line";
        feedback->progress = 0.6;
        goal_handle->publish_feedback(feedback);

        waypoints.push_back(create_pose(tr_x, tr_y, lift_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // Descend
        feedback->status = "descending";
        feedback->progress = 0.7;
        goal_handle->publish_feedback(feedback);

        waypoints.push_back(create_pose(tr_x, tr_y, drawing_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // Draw line to bottom-left
        feedback->status = "drawing_line_2";
        feedback->progress = 0.8;
        goal_handle->publish_feedback(feedback);

        waypoints.push_back(create_pose(bl_x, bl_y, drawing_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // Lift up
        feedback->status = "lifting";
        feedback->progress = 0.9;
        goal_handle->publish_feedback(feedback);

        waypoints.push_back(create_pose(bl_x, bl_y, lift_height));
        if (!execute_cartesian_path(waypoints)) return false;

        return true;
    }

    bool draw_o(double cx, double cy, const std::shared_ptr<GoalHandleDrawShape>& goal_handle,
                std::shared_ptr<DrawShape::Feedback>& feedback) {
        RCLCPP_INFO(node_->get_logger(), "Drawing O at (%.3f, %.3f)", cx, cy);

        double cell_size = node_->get_parameter("cell_size").as_double();
        double drawing_height = node_->get_parameter("drawing_height").as_double();
        double lift_height = node_->get_parameter("lift_height").as_double();
        double x_offset = node_->get_parameter("x_offset").as_double();
        double y_offset = node_->get_parameter("y_offset").as_double();
        double radius = cell_size / 2.0;
        int num_points = 36;
        cx += x_offset;
        cy += y_offset;

        std::vector<geometry_msgs::msg::Pose> waypoints;

        // Move to starting point on circle (lifted)
        double start_x = cx + radius;
        double start_y = cy;

        feedback->status = "moving_to_start";
        feedback->progress = 0.1;
        goal_handle->publish_feedback(feedback);

        waypoints.push_back(create_pose(start_x, start_y, lift_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // Descend to drawing height
        feedback->status = "descending";
        feedback->progress = 0.2;
        goal_handle->publish_feedback(feedback);

        waypoints.push_back(create_pose(start_x, start_y, drawing_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // Draw circle
        feedback->status = "drawing_circle";

        for (int i = 1; i <= num_points; i++) {
            double angle = 2.0 * M_PI * i / num_points;
            double x = cx + radius * std::cos(angle);
            double y = cy + radius * std::sin(angle);

            waypoints.push_back(create_pose(x, y, drawing_height));

            feedback->progress = 0.2 + 0.7 * (static_cast<double>(i) / num_points);
            goal_handle->publish_feedback(feedback);
        }

        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // Lift up
        feedback->status = "lifting";
        feedback->progress = 0.95;
        goal_handle->publish_feedback(feedback);

        waypoints.push_back(create_pose(start_x, start_y, lift_height));
        if (!execute_cartesian_path(waypoints)) return false;

        return true;
    }

    // == Erasing Functions ==
    void execute_erase_grid(const std::shared_ptr<GoalHandleEraseGrid> goal_handle) {
        RCLCPP_INFO(node_->get_logger(), "Executing erase grid goal...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<EraseGrid::Feedback>();
        auto result = std::make_shared<EraseGrid::Result>();

        // Apply constraints
        move_group_->clearPathConstraints();
        if (goal->constraints_identifier != NONE) {
            move_group_->setPathConstraints(set_constraint(goal->constraints_identifier));
        }

        bool success = erase_grid_pattern(goal->cell_poses, goal_handle, feedback);

        if (success) {
            // Return home
            feedback->status = "returning_home";
            feedback->progress = 0.95;
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            if (return_home()) {
                result->success = true;
                result->message = "Grid erased successfully";
                goal_handle->succeed(result);
                RCLCPP_INFO(node_->get_logger(), "Erase goal succeeded!");
            } else {
                result->success = false;
                result->message = "Failed to return home after erasing";
                goal_handle->abort(result);
            }
        } else {
            result->success = false;
            result->message = "Failed to erase grid";
            goal_handle->abort(result);
        }

        // Clear constraints after erasing
        move_group_->clearPathConstraints();
    }

    bool erase_grid_pattern(const std::vector<geometry_msgs::msg::Pose2D>& cell_poses,
                            const std::shared_ptr<GoalHandleEraseGrid>& goal_handle, std::shared_ptr<EraseGrid::Feedback>& feedback) {
        RCLCPP_INFO(node_->get_logger(), "Erasing grid with serpentine pattern");

        // Get parameters
        double drawing_height = node_->get_parameter("drawing_height").as_double();
        double lift_height = node_->get_parameter("lift_height").as_double();
        double x_offset = node_->get_parameter("x_offset").as_double() * -1;  // Invert for erasing
        double y_offset = node_->get_parameter("y_offset").as_double();
        double erase_offset = node_->get_parameter("erase_offset").as_double();
        double grid_theta = cell_poses[0].theta;
        double cos_t = std::cos(grid_theta);
        double sin_t = std::sin(grid_theta);

        // Pattern: TL->TR, TR->MR, MR->ML, ML->BL, BL->BR
        std::vector<geometry_msgs::msg::Pose> waypoints;

        // Helper lambda to transform offset in grid frame to world frame
        auto apply_grid_offset = [&](double cx, double cy, double dx_grid, double dy_grid) -> std::pair<double, double> {
            double world_x = cx + x_offset + (dx_grid * cos_t - dy_grid * sin_t);
            double world_y = cy + y_offset + (dx_grid * sin_t + dy_grid * cos_t);
            return {world_x, world_y};
        };

        // 1. Move to start position (slightly left of top_left center)
        feedback->status = "moving_to_start";
        feedback->progress = 0.05;
        goal_handle->publish_feedback(feedback);

        auto [start_x, start_y] = apply_grid_offset(cell_poses[0].x, cell_poses[0].y, -erase_offset, 0);
        waypoints.push_back(create_pose(start_x, start_y, lift_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // 2. Descend to drawing height
        feedback->status = "descending";
        feedback->progress = 0.1;
        goal_handle->publish_feedback(feedback);

        waypoints.push_back(create_pose(start_x, start_y, drawing_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // 3. TL to TR (slightly right of TR)
        feedback->status = "erasing_row_1";
        feedback->progress = 0.2;
        goal_handle->publish_feedback(feedback);

        auto [tr_x, tr_y] = apply_grid_offset(cell_poses[1].x, cell_poses[1].y, erase_offset, 0);
        waypoints.push_back(create_pose(tr_x, tr_y, drawing_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // 4. TR to MR (at MR center with offset)
        feedback->status = "erasing_row_2_part_1";
        feedback->progress = 0.35;
        goal_handle->publish_feedback(feedback);

        auto [mr_x, mr_y] = apply_grid_offset(cell_poses[2].x, cell_poses[2].y, erase_offset, 0);
        waypoints.push_back(create_pose(mr_x, mr_y, drawing_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // 5. MR to ML
        feedback->status = "erasing_row_2_part_2";
        feedback->progress = 0.5;
        goal_handle->publish_feedback(feedback);

        auto [ml_x, ml_y] = apply_grid_offset(cell_poses[3].x, cell_poses[3].y, -erase_offset, 0);
        waypoints.push_back(create_pose(ml_x, ml_y, drawing_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // 6. ML to BL
        feedback->status = "erasing_row_3_part_1";
        feedback->progress = 0.65;
        goal_handle->publish_feedback(feedback);

        auto [bl_x, bl_y] = apply_grid_offset(cell_poses[4].x, cell_poses[4].y, -erase_offset, 0);
        waypoints.push_back(create_pose(bl_x, bl_y, drawing_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // 7. BL to BR (end slightly right of BR)
        feedback->status = "erasing_row_3_part_2";
        feedback->progress = 0.8;
        goal_handle->publish_feedback(feedback);

        auto [br_x, br_y] = apply_grid_offset(cell_poses[5].x, cell_poses[5].y, erase_offset, 0);
        waypoints.push_back(create_pose(br_x, br_y, drawing_height));
        if (!execute_cartesian_path(waypoints)) return false;
        waypoints.clear();

        // 8. Lift up
        feedback->status = "lifting";
        feedback->progress = 0.9;
        goal_handle->publish_feedback(feedback);

        waypoints.push_back(create_pose(br_x, br_y, lift_height));
        if (!execute_cartesian_path(waypoints)) return false;

        RCLCPP_INFO(node_->get_logger(), "Grid erased successfully");
        return true;
    }

    // == Helper Functions ==
    geometry_msgs::msg::Pose create_pose(double x, double y, double z) {
        // Create pose with fixed downward orientation
        geometry_msgs::msg::Quaternion q;
        q.x = -0.70654;
        q.y = 0.70767;
        q.z = 0.0;
        q.w = 0.0012;

        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation = q;

        return pose;
    }

    bool execute_cartesian_path(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.005;
        const double jump_threshold = 0.0;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.90) {
            RCLCPP_ERROR(node_->get_logger(), "Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
            return false;
        }

        moveit_msgs::msg::RobotTrajectory trajectory_slow;
        trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.05);
        robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_->getName());
        rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);
        iptp.computeTimeStamps(rt, 1, 1);
        rt.getRobotTrajectoryMsg(trajectory);
        iptp.computeTimeStamps(rt, 0.1, 0.1);
        rt.getRobotTrajectoryMsg(trajectory_slow);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory_slow;
        auto result = move_group_->execute(plan);
        return (result == moveit::core::MoveItErrorCode::SUCCESS);
    }

    bool return_home() {
        RCLCPP_INFO(node_->get_logger(), "Returning to home position...");

        auto home_joints = node_->get_parameter("home_joint_positions").as_double_array();

        std::map<std::string, double> joint_targets = {
            {"shoulder_pan_joint", deg2rad(home_joints[0])}, {"shoulder_lift_joint", deg2rad(home_joints[1])},
            {"elbow_joint", deg2rad(home_joints[2])},        {"wrist_1_joint", deg2rad(home_joints[3])},
            {"wrist_2_joint", deg2rad(home_joints[4])},      {"wrist_3_joint", deg2rad(home_joints[5])}};

        move_group_->setJointValueTarget(joint_targets);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            move_group_->execute(plan);
            return true;
        }

        return false;
    }

    void setupCollisionObjects() {
        std::string frame_id = "world";
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 0.02, 0.9, 0.85, -0.18, 0.435, frame_id, "backWall"));
        planning_scene_interface.applyCollisionObject(generateCollisionObject(0.02, 1.5, 0.9, -0.35, 0.57, 0.435, frame_id, "sideWall"));
        planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 1.5, 0.02, 0.85, 0.57, -0.01, frame_id, "table"));
        planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 1.5, 0.02, 0.85, 0.57, 0.88, frame_id, "ceiling"));
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
    rclcpp_action::Server<DrawShape>::SharedPtr draw_action_server_;
    rclcpp_action::Server<EraseGrid>::SharedPtr erase_action_server_;

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
// ros2 service call /moveit_path_plan helper/srv/MoveRequest "{command: 'joint', positions: [0, -74.5, 90, -105, -90, 0]}"

// Home pose Cartesian
// ros2 service call /moveit_path_plan helper/srv/MoveRequest "{command: 'cartesian', positions: [0.59, 0.133, 0.366, 180, 0, 90]}"