#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class GazeboObjectSync : public rclcpp::Node
{
public:
  GazeboObjectSync()
  : Node("gazebo_object_sync"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    object_id_ = declare_parameter<std::string>("object_id", "demo_box");
    planning_frame_ = declare_parameter<std::string>("planning_frame", "base_footprint");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    dimensions_ = declare_parameter<std::vector<double>>(
        "dimensions", { 0.04, 0.04, 0.10 });
    expected_table_z_ = declare_parameter<double>("expected_world_center_z", 1.05);
    table_tolerance_ = declare_parameter<double>("table_height_tolerance", 0.01);
    position_threshold_ = declare_parameter<double>("position_update_threshold", 0.001);
    orientation_threshold_ = declare_parameter<double>("orientation_update_threshold", 0.01);
    table_id_ = declare_parameter<std::string>("table.id", "work_table");
    table_world_position_ = declare_parameter<std::vector<double>>(
        "table.world_position", { 0.75, 0.50, 0.975 });
    table_dimensions_ = declare_parameter<std::vector<double>>(
        "table.dimensions", { 1.50, 1.00, 0.05 });
    if (dimensions_.size() != 3)
      throw std::runtime_error("dimensions must contain exactly 3 values");
    if (table_world_position_.size() != 3 || table_dimensions_.size() != 3)
      throw std::runtime_error("table position and dimensions must contain exactly 3 values");

    // Other things on the tabletop that the arm must plan around.
    for (const auto& name : declare_parameter<std::vector<std::string>>(
             "obstacle_names", std::vector<std::string>{}))
    {
      StaticObstacle obstacle;
      obstacle.id = name;
      obstacle.shape = declare_parameter<std::string>("obstacles." + name + ".shape", "box");
      obstacle.dimensions = declare_parameter<std::vector<double>>(
          "obstacles." + name + ".dimensions", std::vector<double>{});
      obstacle.world_position = declare_parameter<std::vector<double>>(
          "obstacles." + name + ".world_position", std::vector<double>{});
      obstacle.world_yaw = declare_parameter<double>("obstacles." + name + ".world_yaw", 0.0);
      const std::size_t expected = obstacle.shape == "cylinder" ? 2 : 3;
      if (obstacle.dimensions.size() != expected || obstacle.world_position.size() != 3)
        throw std::runtime_error("obstacle '" + name + "' has invalid dimensions/position");
      obstacles_.push_back(obstacle);
    }

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/fr3_demo/gazebo_object_pose", qos);
    object_publisher_ = create_publisher<moveit_msgs::msg::CollisionObject>(
        "/fr3_demo/object_description", qos);
    // Objects now come from the camera (tabletop_perception). Reading the box
    // pose straight from Gazebo is kept only as an opt-in debugging aid.
    if (declare_parameter<bool>("sync_gazebo_object", false))
    {
      pose_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
          "/fr3_demo/gz/pose/demo_box", rclcpp::QoS(10).reliable(),
          [this](geometry_msgs::msg::Pose::ConstSharedPtr message) { receivePose(*message); });
      RCLCPP_INFO(
          get_logger(), "Syncing Gazebo demo_box pose; output frame is '%s'",
          planning_frame_.c_str());
    }
    table_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      addTableIfNeeded();
      if (table_added_)
        table_timer_->cancel();
    });
  }

private:
  void receivePose(const geometry_msgs::msg::Pose& message)
  {
    addTableIfNeeded();

    geometry_msgs::msg::PoseStamped world_pose;
    world_pose.header.frame_id = world_frame_;
    world_pose.header.stamp = now();
    world_pose.pose = message;

    geometry_msgs::msg::PoseStamped planning_pose;
    try
    {
      planning_pose = tf_buffer_.transform(
          world_pose, planning_frame_, tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException& error)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Cannot transform Gazebo pose: %s", error.what());
      return;
    }
    pose_publisher_->publish(planning_pose);

    if (!table_checked_)
    {
      table_checked_ = true;
      const double error = std::abs(world_pose.pose.position.z - expected_table_z_);
      if (error <= table_tolerance_)
        RCLCPP_INFO(
            get_logger(), "demo_box is on the table: world z=%.4f m",
            world_pose.pose.position.z);
      else
        RCLCPP_WARN(
            get_logger(), "demo_box centre z=%.4f m, expected %.4f +/- %.4f m",
            world_pose.pose.position.z, expected_table_z_, table_tolerance_);
    }

    if (scene_.getAttachedObjects({ object_id_ }).count(object_id_) != 0)
      return;
    if (have_last_pose_ &&
        positionDistance(planning_pose.pose, last_pose_) < position_threshold_ &&
        orientationDistance(planning_pose.pose, last_pose_) < orientation_threshold_)
      return;

    moveit_msgs::msg::CollisionObject object;
    object.header = planning_pose.header;
    object.id = object_id_;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.resize(3);
    std::copy(dimensions_.begin(), dimensions_.end(), box.dimensions.begin());
    object.primitives.push_back(box);
    object.primitive_poses.push_back(planning_pose.pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    object_publisher_->publish(object);

    if (!scene_.applyCollisionObject(object))
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "MoveIt rejected synchronized demo_box pose");
      return;
    }
    last_pose_ = planning_pose.pose;
    have_last_pose_ = true;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Synced demo_box: base_footprint [%.3f, %.3f, %.3f]",
        planning_pose.pose.position.x, planning_pose.pose.position.y,
        planning_pose.pose.position.z);
  }

  void addTableIfNeeded()
  {
    if (table_added_)
      return;
    geometry_msgs::msg::PoseStamped world_pose;
    world_pose.header.frame_id = world_frame_;
    world_pose.header.stamp = now();
    world_pose.pose.position.x = table_world_position_[0];
    world_pose.pose.position.y = table_world_position_[1];
    world_pose.pose.position.z = table_world_position_[2];
    world_pose.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped planning_pose;
    try
    {
      planning_pose = tf_buffer_.transform(
          world_pose, planning_frame_, tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException&)
    {
      return;
    }

    moveit_msgs::msg::CollisionObject table;
    table.header = planning_pose.header;
    table.id = table_id_;
    shape_msgs::msg::SolidPrimitive slab;
    slab.type = shape_msgs::msg::SolidPrimitive::BOX;
    slab.dimensions.resize(3);
    std::copy(table_dimensions_.begin(), table_dimensions_.end(), slab.dimensions.begin());
    table.primitives.push_back(slab);
    table.primitive_poses.push_back(planning_pose.pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;
    if (!scene_.applyCollisionObject(table))
      return;
    table_added_ = true;
    addObstacles();
    RCLCPP_INFO(
        get_logger(), "Added MoveIt table collision '%s' (top at world z=1.0 m)",
        table_id_.c_str());
  }

  void addObstacles()
  {
    for (const auto& obstacle : obstacles_)
    {
      geometry_msgs::msg::PoseStamped world_pose;
      world_pose.header.frame_id = world_frame_;
      world_pose.pose.position.x = obstacle.world_position[0];
      world_pose.pose.position.y = obstacle.world_position[1];
      world_pose.pose.position.z = obstacle.world_position[2];
      world_pose.pose.orientation.z = std::sin(obstacle.world_yaw / 2.0);
      world_pose.pose.orientation.w = std::cos(obstacle.world_yaw / 2.0);
      geometry_msgs::msg::PoseStamped planning_pose;
      try
      {
        planning_pose = tf_buffer_.transform(
            world_pose, planning_frame_, tf2::durationFromSec(0.5));
      }
      catch (const tf2::TransformException& error)
      {
        RCLCPP_ERROR(get_logger(), "Cannot place obstacle '%s': %s",
                     obstacle.id.c_str(), error.what());
        continue;
      }
      moveit_msgs::msg::CollisionObject object;
      object.header = planning_pose.header;
      object.id = obstacle.id;
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = obstacle.shape == "cylinder" ? shape_msgs::msg::SolidPrimitive::CYLINDER
                                                    : shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions.assign(obstacle.dimensions.begin(), obstacle.dimensions.end());
      object.primitives.push_back(primitive);
      object.primitive_poses.push_back(planning_pose.pose);
      object.operation = moveit_msgs::msg::CollisionObject::ADD;
      if (scene_.applyCollisionObject(object))
        RCLCPP_INFO(get_logger(), "Added MoveIt obstacle '%s'", obstacle.id.c_str());
    }
  }

  static double positionDistance(
      const geometry_msgs::msg::Pose& first, const geometry_msgs::msg::Pose& second)
  {
    const double dx = first.position.x - second.position.x;
    const double dy = first.position.y - second.position.y;
    const double dz = first.position.z - second.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  static double orientationDistance(
      const geometry_msgs::msg::Pose& first, const geometry_msgs::msg::Pose& second)
  {
    const auto& a = first.orientation;
    const auto& b = second.orientation;
    const double dot = std::clamp(
        std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w), 0.0, 1.0);
    return 2.0 * std::acos(dot);
  }

  std::string object_id_;
  std::string planning_frame_;
  std::string world_frame_;
  std::string table_id_;
  std::vector<double> dimensions_;
  std::vector<double> table_world_position_;
  std::vector<double> table_dimensions_;
  double expected_table_z_;
  double table_tolerance_;
  double position_threshold_;
  double orientation_threshold_;
  bool table_checked_{ false };
  bool table_added_{ false };

  struct StaticObstacle
  {
    std::string id;
    std::string shape;  // "box" [x, y, z] or "cylinder" [height, radius]
    std::vector<double> dimensions;
    std::vector<double> world_position;
    double world_yaw{ 0.0 };
  };
  std::vector<StaticObstacle> obstacles_;
  bool have_last_pose_{ false };
  geometry_msgs::msg::Pose last_pose_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::TimerBase::SharedPtr table_timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr object_publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<GazeboObjectSync>());
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("gazebo_object_sync"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
