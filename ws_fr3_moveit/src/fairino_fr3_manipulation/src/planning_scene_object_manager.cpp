#include <memory>
#include <mutex>
#include <string>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class PlanningSceneObjectManager : public rclcpp::Node
{
public:
  PlanningSceneObjectManager()
  : Node("planning_scene_object_manager")
  {
    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                                 .reliable()
                                 .transient_local();
    object_subscription_ = create_subscription<moveit_msgs::msg::CollisionObject>(
        "/fr3_demo/object_description", latched_qos,
        [this](const moveit_msgs::msg::CollisionObject::SharedPtr message) {
          receiveObjectDescription(*message);
        });

    add_service_ = create_service<std_srvs::srv::Trigger>(
        "/fr3_demo/add_object",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          addObject(response);
        });
    remove_service_ = create_service<std_srvs::srv::Trigger>(
        "/fr3_demo/remove_object",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          removeObject(response);
        });

    RCLCPP_INFO(get_logger(), "Waiting for /fr3_demo/object_description");
    RCLCPP_INFO(get_logger(), "Add service: /fr3_demo/add_object");
    RCLCPP_INFO(get_logger(), "Remove service: /fr3_demo/remove_object");
    RCLCPP_INFO(get_logger(), "No object is added automatically.");
  }

private:
  void receiveObjectDescription(const moveit_msgs::msg::CollisionObject& message)
  {
    if (message.id.empty() || message.header.frame_id.empty() ||
        message.primitives.empty() || message.primitive_poses.empty())
    {
      RCLCPP_ERROR(get_logger(), "Rejected incomplete object description");
      return;
    }

    std::lock_guard<std::mutex> lock(object_mutex_);
    const bool first_description = !has_object_;
    latest_object_ = message;
    latest_object_.operation = moveit_msgs::msg::CollisionObject::ADD;
    has_object_ = true;
    if (first_description)
      RCLCPP_INFO(
          get_logger(), "Received object '%s' in frame '%s'; call /fr3_demo/add_object when ready",
          latest_object_.id.c_str(), latest_object_.header.frame_id.c_str());
  }

  bool copyLatestObject(moveit_msgs::msg::CollisionObject& object)
  {
    std::lock_guard<std::mutex> lock(object_mutex_);
    if (!has_object_)
      return false;
    object = latest_object_;
    return true;
  }

  void addObject(const std_srvs::srv::Trigger::Response::SharedPtr& response)
  {
    moveit_msgs::msg::CollisionObject object;
    if (!copyLatestObject(object))
    {
      response->success = false;
      response->message = "No object description received; start object_visualization.launch.py first";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    object.header.stamp = now();
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    response->success = planning_scene_interface_.applyCollisionObject(object);
    response->message = response->success ?
        "Added '" + object.id + "' to the MoveIt Planning Scene" :
        "MoveIt rejected object '" + object.id + "'";
    if (response->success)
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    else
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }

  void removeObject(const std_srvs::srv::Trigger::Response::SharedPtr& response)
  {
    moveit_msgs::msg::CollisionObject object;
    if (!copyLatestObject(object))
    {
      response->success = false;
      response->message = "No object description received; object ID is unknown";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    moveit_msgs::msg::CollisionObject removal;
    removal.header.frame_id = object.header.frame_id;
    removal.header.stamp = now();
    removal.id = object.id;
    removal.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    response->success = planning_scene_interface_.applyCollisionObject(removal);
    response->message = response->success ?
        "Removed '" + object.id + "' from the MoveIt Planning Scene; RViz Marker remains" :
        "MoveIt rejected removal of '" + object.id + "'";
    if (response->success)
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    else
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr object_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr add_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_service_;
  std::mutex object_mutex_;
  moveit_msgs::msg::CollisionObject latest_object_;
  bool has_object_{ false };
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningSceneObjectManager>());
  rclcpp::shutdown();
  return 0;
}
