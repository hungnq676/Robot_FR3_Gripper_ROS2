#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using ErrorCode = moveit::core::MoveItErrorCode;
using namespace std::chrono_literals;

bool succeeded(const ErrorCode& result)
{
  return result == ErrorCode::SUCCESS;
}
}  // namespace

class PickAndCarry
{
public:
  explicit PickAndCarry(const rclcpp::Node::SharedPtr& node)
  : node_(node), logger_(node->get_logger()),
    arm_(node, node->declare_parameter<std::string>("arm_group", "arm")),
    gripper_(node, node->declare_parameter<std::string>("gripper_group", "gripper"))
  {
    tcp_link_ = node_->declare_parameter<std::string>("tcp_link", "grasp_frame");
    object_id_ = node_->declare_parameter<std::string>("object_id", "demo_box");
    touch_links_ = node_->declare_parameter<std::vector<std::string>>(
        "touch_links",
        { "wrist3_link", "base_link_gripper", "clamp_1", "clamp_2", "grasp_frame" });
    place_frame_ = node_->declare_parameter<std::string>("place.frame", "base_footprint");
    const auto place_position = node_->declare_parameter<std::vector<double>>(
        "place.position", { 0.30, 0.25, 0.30 });
    const auto place_orientation = node_->declare_parameter<std::vector<double>>(
        "place.orientation", { 1.0, 0.0, 0.0, 0.0 });
    lift_distance_ = node_->declare_parameter<double>("lift_distance", 0.15);
    cartesian_step_ = node_->declare_parameter<double>("cartesian_step", 0.005);
    min_fraction_ = node_->declare_parameter<double>("minimum_cartesian_fraction", 0.95);
    velocity_ = node_->declare_parameter<double>("velocity_scaling", 0.12);
    acceleration_ = node_->declare_parameter<double>("acceleration_scaling", 0.10);
    planning_time_ = node_->declare_parameter<double>("planning_time", 10.0);
    gazebo_attach_timeout_ = node_->declare_parameter<double>("gazebo_attach_timeout", 5.0);
    gripper_grasp_positions_ = node_->declare_parameter<std::vector<double>>(
        "gripper_grasp_positions", std::vector<double>{});

    if (place_position.size() != 3 || place_orientation.size() != 4)
      throw std::runtime_error("place.position must have 3 and place.orientation 4 values");
    place_pose_.header.frame_id = place_frame_;
    place_pose_.pose.position.x = place_position[0];
    place_pose_.pose.position.y = place_position[1];
    place_pose_.pose.position.z = place_position[2];
    place_pose_.pose.orientation.x = place_orientation[0];
    place_pose_.pose.orientation.y = place_orientation[1];
    place_pose_.pose.orientation.z = place_orientation[2];
    place_pose_.pose.orientation.w = place_orientation[3];

    arm_.setEndEffectorLink(tcp_link_);
    arm_.setPlanningTime(planning_time_);
    arm_.setMaxVelocityScalingFactor(velocity_);
    arm_.setMaxAccelerationScalingFactor(acceleration_);
    gripper_.setPlanningTime(5.0);
    gripper_.setMaxVelocityScalingFactor(velocity_);
    gripper_.setMaxAccelerationScalingFactor(acceleration_);

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    object_subscription_ = node_->create_subscription<moveit_msgs::msg::CollisionObject>(
        "/fr3_demo/object_description", qos,
        [this](moveit_msgs::msg::CollisionObject::ConstSharedPtr msg) {
          object_ = *msg;
          have_object_ = true;
        });
    grasp_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/fr3_demo/grasp_pose", qos,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          grasp_pose_ = *msg;
          have_grasp_ = true;
        });
    pregrasp_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/fr3_demo/pregrasp_pose", qos,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          pregrasp_pose_ = *msg;
          have_pregrasp_ = true;
        });
    gazebo_attachment_subscription_ = node_->create_subscription<std_msgs::msg::String>(
        "/fr3_demo/gazebo_attachment_state", rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          // DetachableJoint publishes "attached" or "detached".
          gazebo_attached_ = (msg->data == "attached");
          have_gazebo_attachment_state_ = true;
        });
    gazebo_attach_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(
        "/fr3_demo/gazebo_attach", rclcpp::QoS(1).reliable());
    gazebo_detach_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(
        "/fr3_demo/gazebo_detach", rclcpp::QoS(1).reliable());

    get_scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
        "/get_planning_scene");
    service_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/fr3_demo/run_pick_and_carry",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          run(response);
        }, rmw_qos_profile_services_default, service_callback_group_);

    RCLCPP_INFO(logger_, "Ready. Call /fr3_demo/run_pick_and_carry to execute stages 4-7.");
    RCLCPP_WARN(logger_, "The sequence moves the simulated robot and finishes with the object attached.");
  }

private:
  // Gazebo tracks trajectories with a lag; wait until the arm stops moving so the
  // next trajectory starts within MoveIt's allowed_start_tolerance.
  void waitUntilSettled(MoveGroup& group)
  {
    std::vector<double> previous = group.getCurrentJointValues();
    for (int i = 0; i < 30; ++i)
    {
      std::this_thread::sleep_for(100ms);
      const std::vector<double> current = group.getCurrentJointValues();
      double max_change = 0.0;
      for (std::size_t j = 0; j < current.size() && j < previous.size(); ++j)
        max_change = std::max(max_change, std::abs(current[j] - previous[j]));
      if (max_change < 1e-4)
        return;
      previous = current;
    }
    RCLCPP_WARN(logger_, "Arm did not fully settle within 3 s; continuing");
  }

  // "closed" is far narrower than the box; driving the jaws there squeezes and
  // flips the box. Close only to the configured box width when it is set.
  bool closeGripperForGrasp()
  {
    if (gripper_grasp_positions_.empty())
      return moveNamed(gripper_, "closed");
    waitUntilSettled(gripper_);
    gripper_.setStartStateToCurrentState();
    if (!gripper_.setJointValueTarget(gripper_grasp_positions_))
      return false;
    MoveGroup::Plan plan;
    return succeeded(gripper_.plan(plan)) && succeeded(gripper_.execute(plan));
  }

  bool moveNamed(MoveGroup& group, const std::string& target)
  {
    waitUntilSettled(group);
    group.setStartStateToCurrentState();
    if (!group.setNamedTarget(target))
      return false;
    MoveGroup::Plan plan;
    return succeeded(group.plan(plan)) && succeeded(group.execute(plan));
  }

  bool movePose(const geometry_msgs::msg::PoseStamped& target, const std::string& label)
  {
    waitUntilSettled(arm_);
    arm_.setStartStateToCurrentState();
    arm_.setPoseTarget(target, tcp_link_);
    MoveGroup::Plan plan;
    const bool planned = succeeded(arm_.plan(plan));
    arm_.clearPoseTargets();
    if (!planned)
    {
      RCLCPP_ERROR(logger_, "Planning to %s failed", label.c_str());
      return false;
    }
    if (!succeeded(arm_.execute(plan)))
    {
      RCLCPP_ERROR(logger_, "Execution to %s failed", label.c_str());
      return false;
    }
    return true;
  }

  bool moveCartesian(const geometry_msgs::msg::Pose& target, const std::string& label)
  {
    waitUntilSettled(arm_);
    arm_.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = arm_.computeCartesianPath(
        { target }, cartesian_step_, 0.0, trajectory, true);
    RCLCPP_INFO(logger_, "%s Cartesian path: %.1f%%", label.c_str(), fraction * 100.0);
    if (fraction < min_fraction_)
      return false;
    return succeeded(arm_.execute(trajectory));
  }

  bool getAndAllowObjectContact(moveit_msgs::msg::AllowedCollisionMatrix& original)
  {
    if (!get_scene_client_->wait_for_service(3s))
    {
      RCLCPP_ERROR(logger_, "/get_planning_scene is unavailable");
      return false;
    }
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = get_scene_client_->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready)
    {
      RCLCPP_ERROR(logger_, "Timed out reading Allowed Collision Matrix");
      return false;
    }
    original = future.get()->scene.allowed_collision_matrix;
    collision_detection::AllowedCollisionMatrix matrix(original);
    matrix.setEntry(object_id_, touch_links_, true);
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    matrix.getMessage(diff.allowed_collision_matrix);
    if (!scene_.applyPlanningScene(diff))
    {
      RCLCPP_ERROR(logger_, "Could not apply temporary gripper/object contact permission");
      return false;
    }
    return true;
  }

  void restoreCollisionMatrix(const moveit_msgs::msg::AllowedCollisionMatrix& original)
  {
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.allowed_collision_matrix = original;
    if (!scene_.applyPlanningScene(diff))
      RCLCPP_WARN(logger_, "Could not restore the original Allowed Collision Matrix");
  }

  void fail(const std::string& message,
            const std_srvs::srv::Trigger::Response::SharedPtr& response)
  {
    response->success = false;
    response->message = message;
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  }

  bool commandGazeboAttachment(bool attach)
  {
    const auto publisher = attach ? gazebo_attach_publisher_ : gazebo_detach_publisher_;
    if (publisher->get_subscription_count() == 0)
    {
      RCLCPP_ERROR(
          logger_, "Gazebo attachment bridge is unavailable; run gazebo_object_sync.launch.py");
      return false;
    }
    std_msgs::msg::Empty command;
    publisher->publish(command);
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(gazebo_attach_timeout_);
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (have_gazebo_attachment_state_ && gazebo_attached_ == attach)
        return true;
      std::this_thread::sleep_for(50ms);
    }
    RCLCPP_ERROR(
        logger_, "Gazebo did not confirm physical %s within %.1f seconds",
        attach ? "attach" : "detach", gazebo_attach_timeout_);
    return false;
  }

  void run(const std_srvs::srv::Trigger::Response::SharedPtr& response)
  {
    if (busy_.exchange(true))
    {
      fail("Pick-and-carry is already running", response);
      return;
    }
    struct BusyReset
    {
      std::atomic_bool& busy;
      ~BusyReset() { busy = false; }
    } reset{ busy_ };

    if (!have_object_ || !have_grasp_ || !have_pregrasp_)
    {
      fail("Missing object/grasp topics. Run Steps 1 and 3 first", response);
      return;
    }
    const auto known = scene_.getKnownObjectNames();
    if (std::find(known.begin(), known.end(), object_id_) == known.end())
    {
      fail("demo_box is not in the Planning Scene. Call Step 2 add_object first", response);
      return;
    }

    RCLCPP_INFO(logger_, "Stage 4.1: opening gripper");
    if (!moveNamed(gripper_, "open"))
    {
      fail("Could not open gripper", response);
      return;
    }
    RCLCPP_INFO(logger_, "Stage 4.2: planning and executing to pregrasp");
    if (!movePose(pregrasp_pose_, "pregrasp"))
    {
      fail("Could not reach pregrasp", response);
      return;
    }

    moveit_msgs::msg::AllowedCollisionMatrix original_acm;
    if (!getAndAllowObjectContact(original_acm))
    {
      fail("Could not configure controlled object contact", response);
      return;
    }

    RCLCPP_INFO(logger_, "Stage 5: Cartesian approach to grasp");
    if (!moveCartesian(grasp_pose_.pose, "Approach"))
    {
      restoreCollisionMatrix(original_acm);
      fail("Cartesian approach failed", response);
      return;
    }

    RCLCPP_INFO(logger_, "Stage 6.1: closing gripper");
    if (!closeGripperForGrasp())
    {
      restoreCollisionMatrix(original_acm);
      fail("Could not close gripper", response);
      return;
    }
    RCLCPP_INFO(logger_, "Stage 6.2: physically attaching Gazebo demo_box");
    if (!commandGazeboAttachment(true))
    {
      restoreCollisionMatrix(original_acm);
      fail("Gazebo physical attach failed; object was not lifted", response);
      return;
    }

    RCLCPP_INFO(logger_, "Stage 6.3: attaching MoveIt '%s' to '%s'",
                object_id_.c_str(), tcp_link_.c_str());
    if (!arm_.attachObject(object_id_, tcp_link_, touch_links_))
    {
      commandGazeboAttachment(false);
      restoreCollisionMatrix(original_acm);
      fail("Could not attach object", response);
      return;
    }
    std::this_thread::sleep_for(500ms);
    restoreCollisionMatrix(original_acm);

    RCLCPP_INFO(logger_, "Stage 7.1: lifting attached object by %.3f m", lift_distance_);
    auto lift_pose = arm_.getCurrentPose(tcp_link_).pose;
    lift_pose.position.z += lift_distance_;
    if (!moveCartesian(lift_pose, "Lift"))
    {
      fail("Lift failed; object remains attached for safety", response);
      return;
    }

    RCLCPP_INFO(logger_, "Stage 7.2: carrying object to place_prepose");
    place_pose_.header.stamp = node_->now();
    if (!movePose(place_pose_, "place_prepose"))
    {
      fail("Carry plan failed; object remains attached for safety", response);
      return;
    }

    response->success = true;
    response->message = "Stages 4-7 completed; demo_box is attached at place_prepose";
    RCLCPP_INFO(logger_, "%s", response->message.c_str());
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  MoveGroup arm_;
  MoveGroup gripper_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  std::string tcp_link_;
  std::string object_id_;
  std::string place_frame_;
  std::vector<std::string> touch_links_;
  double lift_distance_;
  double cartesian_step_;
  double min_fraction_;
  double velocity_;
  double acceleration_;
  double planning_time_;
  double gazebo_attach_timeout_;
  geometry_msgs::msg::PoseStamped grasp_pose_;
  geometry_msgs::msg::PoseStamped pregrasp_pose_;
  geometry_msgs::msg::PoseStamped place_pose_;
  moveit_msgs::msg::CollisionObject object_;
  std::atomic_bool have_object_{ false };
  std::atomic_bool have_grasp_{ false };
  std::atomic_bool have_pregrasp_{ false };
  std::atomic_bool busy_{ false };
  std::atomic_bool gazebo_attached_{ false };
  std::atomic_bool have_gazebo_attachment_state_{ false };
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr object_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pregrasp_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gazebo_attachment_subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr gazebo_attach_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr gazebo_detach_publisher_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  std::vector<double> gripper_grasp_positions_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pick_and_carry");
  try
  {
    auto pipeline = std::make_shared<PickAndCarry>(node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
