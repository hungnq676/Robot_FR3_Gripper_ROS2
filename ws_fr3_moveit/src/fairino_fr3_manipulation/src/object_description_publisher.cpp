#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <visualization_msgs/msg/marker.hpp>

class ObjectDescriptionPublisher : public rclcpp::Node
{
public:
  ObjectDescriptionPublisher()
  : Node("object_description_publisher")
  {
    object_id_ = declare_parameter<std::string>("object.id", "demo_box");
    frame_id_ = declare_parameter<std::string>("object.frame_id", "base_footprint");
    shape_ = declare_parameter<std::string>("object.shape", "box");
    dimensions_ = declare_parameter<std::vector<double>>(
        "object.dimensions", { 0.04, 0.04, 0.10 });
    position_ = declare_parameter<std::vector<double>>(
        "object.position", { 0.40, -0.15, 0.05 });
    orientation_ = declare_parameter<std::vector<double>>(
        "object.orientation", { 0.0, 0.0, 0.0, 1.0 });
    color_ = declare_parameter<std::vector<double>>(
        "object.color", { 0.10, 0.80, 0.20, 1.0 });
    const double publish_rate = declare_parameter<double>("publish_rate", 1.0);

    validateParameters(publish_rate);

    // A late subscriber immediately receives the latest object description.
    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                                 .reliable()
                                 .transient_local();
    description_publisher_ = create_publisher<moveit_msgs::msg::CollisionObject>(
        "/fr3_demo/object_description", latched_qos);
    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
        "/fr3_demo/object_marker", latched_qos);

    publishObject();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate),
        std::bind(&ObjectDescriptionPublisher::publishObject, this));

    RCLCPP_INFO(
        get_logger(),
        "Publishing box '%s' in '%s': size=[%.3f, %.3f, %.3f] m, "
        "position=[%.3f, %.3f, %.3f] m",
        object_id_.c_str(), frame_id_.c_str(), dimensions_[0], dimensions_[1],
        dimensions_[2], position_[0], position_[1], position_[2]);
    RCLCPP_INFO(get_logger(), "Description topic: /fr3_demo/object_description");
    RCLCPP_INFO(get_logger(), "RViz marker topic: /fr3_demo/object_marker");
    RCLCPP_INFO(get_logger(), "This node does not add the object to the MoveIt Planning Scene.");
  }

private:
  void validateParameters(double publish_rate)
  {
    if (shape_ != "box")
      throw std::runtime_error("Step 1 currently supports only object.shape='box'");
    if (dimensions_.size() != 3 || position_.size() != 3 ||
        orientation_.size() != 4 || color_.size() != 4)
      throw std::runtime_error(
          "Expected dimensions[3], position[3], orientation[4] and color[4]");
    if (std::any_of(dimensions_.begin(), dimensions_.end(),
                    [](double value) { return value <= 0.0; }))
      throw std::runtime_error("Every box dimension must be greater than zero");
    const double quaternion_norm = std::sqrt(
        orientation_[0] * orientation_[0] + orientation_[1] * orientation_[1] +
        orientation_[2] * orientation_[2] + orientation_[3] * orientation_[3]);
    if (quaternion_norm < 1e-9)
      throw std::runtime_error("Object orientation quaternion must not be zero");
    for (double& value : orientation_)
      value /= quaternion_norm;
    for (double& value : color_)
      value = std::clamp(value, 0.0, 1.0);
    if (publish_rate <= 0.0)
      throw std::runtime_error("publish_rate must be greater than zero");
  }

  geometry_msgs::msg::Pose objectPose() const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = position_[0];
    pose.position.y = position_[1];
    pose.position.z = position_[2];
    pose.orientation.x = orientation_[0];
    pose.orientation.y = orientation_[1];
    pose.orientation.z = orientation_[2];
    pose.orientation.w = orientation_[3];
    return pose;
  }

  void publishObject()
  {
    const auto stamp = now();

    moveit_msgs::msg::CollisionObject description;
    description.header.frame_id = frame_id_;
    description.header.stamp = stamp;
    description.id = object_id_;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.resize(3);
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = dimensions_[0];
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = dimensions_[1];
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = dimensions_[2];
    description.primitives.push_back(box);
    description.primitive_poses.push_back(objectPose());
    description.operation = moveit_msgs::msg::CollisionObject::ADD;
    description_publisher_->publish(description);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "fr3_demo_objects";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = objectPose();
    marker.scale.x = dimensions_[0];
    marker.scale.y = dimensions_[1];
    marker.scale.z = dimensions_[2];
    marker.color.r = static_cast<float>(color_[0]);
    marker.color.g = static_cast<float>(color_[1]);
    marker.color.b = static_cast<float>(color_[2]);
    marker.color.a = static_cast<float>(color_[3]);
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    marker_publisher_->publish(marker);
  }

  std::string object_id_;
  std::string frame_id_;
  std::string shape_;
  std::vector<double> dimensions_;
  std::vector<double> position_;
  std::vector<double> orientation_;
  std::vector<double> color_;
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr description_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<ObjectDescriptionPublisher>());
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("object_description_publisher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
