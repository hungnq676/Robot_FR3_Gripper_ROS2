#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

bool moveToNamedTarget(MoveGroupInterface& group, const std::string& target,
                       const rclcpp::Logger& logger)
{
  group.setStartStateToCurrentState();
  if (!group.setNamedTarget(target))
  {
    RCLCPP_ERROR(logger, "Named target '%s' does not exist", target.c_str());
    return false;
  }

  MoveGroupInterface::Plan plan;
  if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Could not plan to named target '%s'", target.c_str());
    return false;
  }
  if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Could not execute named target '%s'", target.c_str());
    return false;
  }
  return true;
}

void pauseFor(double seconds)
{
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>(
      "attached_object_demo",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto logger = node->get_logger();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const auto arm_group_name = node->get_parameter("arm_group").as_string();
  const auto gripper_group_name = node->get_parameter("gripper_group").as_string();
  const auto tcp_link = node->get_parameter("tcp_link").as_string();
  const auto object_id = node->get_parameter("object_id").as_string();
  const auto object_frame = node->get_parameter("object_frame").as_string();
  const auto dimensions = node->get_parameter("object_dimensions").as_double_array();
  const auto position = node->get_parameter("object_position").as_double_array();
  const auto touch_links = node->get_parameter("touch_links").as_string_array();
  const double lift_distance = node->get_parameter("lift_distance").as_double();
  const double cartesian_step = node->get_parameter("cartesian_step").as_double();
  const double minimum_cartesian_fraction =
      node->get_parameter("minimum_cartesian_fraction").as_double();
  const double velocity = node->get_parameter("velocity_scaling").as_double();
  const double acceleration = node->get_parameter("acceleration_scaling").as_double();
  const double stage_pause = node->get_parameter("stage_pause_seconds").as_double();

  if (dimensions.size() != 3 || position.size() != 3)
  {
    RCLCPP_ERROR(logger, "object_dimensions and object_position must each contain 3 values");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  MoveGroupInterface arm(node, arm_group_name);
  MoveGroupInterface gripper(node, gripper_group_name);
  moveit::planning_interface::PlanningSceneInterface scene;

  arm.setEndEffectorLink(tcp_link);
  arm.setMaxVelocityScalingFactor(velocity);
  arm.setMaxAccelerationScalingFactor(acceleration);
  arm.setPlanningTime(10.0);
  gripper.setMaxVelocityScalingFactor(velocity);
  gripper.setMaxAccelerationScalingFactor(acceleration);
  gripper.setPlanningTime(5.0);

  // A previous interrupted run may have left this object attached in the
  // shared planning scene. Always start the demo from a clean state.
  arm.detachObject(object_id);
  scene.removeCollisionObjects({ object_id });
  pauseFor(0.5);

  RCLCPP_INFO(logger, "Stage 1/6: opening the gripper");
  if (!moveToNamedTarget(gripper, "open", logger))
  {
    rclcpp::shutdown();
    spinner.join();
    return 2;
  }

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = object_frame;
  object.id = object_id;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = dimensions[0];
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = dimensions[1];
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = dimensions[2];
  geometry_msgs::msg::Pose object_pose;
  object_pose.orientation.w = 1.0;
  object_pose.position.x = position[0];
  object_pose.position.y = position[1];
  object_pose.position.z = position[2];
  object.primitives.push_back(box);
  object.primitive_poses.push_back(object_pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  RCLCPP_INFO(logger, "Stage 2/6: adding '%s' to the RViz planning scene", object_id.c_str());
  if (!scene.applyCollisionObject(object))
  {
    RCLCPP_ERROR(logger, "MoveIt rejected collision object '%s'", object_id.c_str());
    rclcpp::shutdown();
    spinner.join();
    return 3;
  }
  pauseFor(stage_pause);

  RCLCPP_INFO(logger, "Stage 3/6: attaching '%s' to '%s'", object_id.c_str(), tcp_link.c_str());
  if (!arm.attachObject(object_id, tcp_link, touch_links))
  {
    RCLCPP_ERROR(logger, "Could not attach '%s'", object_id.c_str());
    rclcpp::shutdown();
    spinner.join();
    return 4;
  }
  pauseFor(stage_pause);

  RCLCPP_INFO(logger, "Stage 4/6: closing the gripper");
  if (!moveToNamedTarget(gripper, "closed", logger))
  {
    arm.detachObject(object_id);
    rclcpp::shutdown();
    spinner.join();
    return 5;
  }

  RCLCPP_INFO(logger, "Stage 5/6: lifting the attached object by %.3f m", lift_distance);
  geometry_msgs::msg::Pose lift_pose = arm.getCurrentPose(tcp_link).pose;
  lift_pose.position.z += lift_distance;
  arm.setStartStateToCurrentState();
  moveit_msgs::msg::RobotTrajectory lift_trajectory;
  const double fraction = arm.computeCartesianPath(
      { lift_pose }, cartesian_step, 0.0, lift_trajectory, true);
  RCLCPP_INFO(logger, "Cartesian lift path: %.1f%%", fraction * 100.0);
  const bool lift_executed = fraction >= minimum_cartesian_fraction &&
      arm.execute(lift_trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
  if (!lift_executed)
  {
    RCLCPP_ERROR(logger,
                 "Cartesian lift failed (required %.1f%%, obtained %.1f%%); cleaning up the RViz object",
                 minimum_cartesian_fraction * 100.0, fraction * 100.0);
    arm.detachObject(object_id);
    pauseFor(0.5);
    scene.removeCollisionObjects({ object_id });
    pauseFor(0.5);
    moveToNamedTarget(gripper, "open", logger);
    rclcpp::shutdown();
    spinner.join();
    return 6;
  }
  pauseFor(stage_pause);

  RCLCPP_INFO(logger, "Stage 6/6: detaching the object and opening the gripper");
  if (!arm.detachObject(object_id))
  {
    RCLCPP_ERROR(logger, "Could not detach '%s'", object_id.c_str());
  }
  pauseFor(0.5);
  const bool opened = moveToNamedTarget(gripper, "open", logger);

  RCLCPP_INFO(logger, "Demo finished. '%s' remains in the RViz planning scene.", object_id.c_str());
  rclcpp::shutdown();
  spinner.join();
  return opened ? 0 : 7;
}
