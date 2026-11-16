#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/multi_planner.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/task.h>
#include <fairino_fr3_msgs/msg/finger_forces.hpp>
#include <fairino_fr3_msgs/msg/perceived_object_array.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mtc = moveit::task_constructor;
using namespace std::chrono_literals;

class MtcPickTaskPlanner
{
public:
  explicit MtcPickTaskPlanner(const rclcpp::Node::SharedPtr& node)
  : node_(node), logger_(node->get_logger())
  {
    node_->get_parameter_or("place.release_into_box", release_into_box_, false);
    node_->get_parameter_or("grasp.external_tcp", external_tcp_, std::vector<double>{});
    if ((release_into_box_ || !external_tcp_.empty()) &&
        (!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) != "34"))
      throw std::runtime_error("External grasp/container demo requires simulation domain 34");
    if (!external_tcp_.empty() && external_tcp_.size() != 7)
      throw std::runtime_error("grasp.external_tcp requires xyz + quaternion xyzw");
    node_->get_parameter_or("arm_group", arm_group_, std::string("arm"));
    node_->get_parameter_or("hand_group", hand_group_, std::string("gripper"));
    node_->get_parameter_or("eef", eef_, std::string("custom_gripper"));
    node_->get_parameter_or("tcp_link", tcp_link_, std::string("grasp_frame"));
    node_->get_parameter_or("support_id", support_id_, std::string("work_table"));
    node_->get_parameter_or("mount_link", mount_link_, std::string("base_link"));
    node_->get_parameter_or(
        "touch_links", touch_links_,
        std::vector<std::string>{ "base_link_gripper", "clamp_1", "clamp_2" });
    node_->get_parameter_or("world_frame", world_frame_, std::string("base_footprint"));
    node_->get_parameter_or("cylinder_grasp_angle_delta", cylinder_angle_delta_, M_PI / 4.0);
    node_->get_parameter_or("gripper.jaw_gap_at_zero", jaw_gap_at_zero_, 0.070);
    node_->get_parameter_or("gripper.joint_limit", gripper_joint_limit_, 0.036);
    node_->get_parameter_or("gripper.squeeze", grasp_squeeze_, 0.0002);
    node_->get_parameter_or("gripper.tcp_to_palm", tcp_to_palm_, 0.0235);
    node_->get_parameter_or("gripper.tcp_to_fingertip", tcp_to_fingertip_, 0.040);
    node_->get_parameter_or("gripper.palm_clearance", palm_clearance_, 0.008);
    node_->get_parameter_or("gripper.table_clearance", table_clearance_, 0.005);
    node_->get_parameter_or("gripper.min_finger_contact", min_finger_contact_, 0.015);
    node_->get_parameter_or("planner.ptp_first", ptp_first_, true);
    node_->get_parameter_or("place.yaw_steps", place_yaw_steps_, 12);
    node_->get_parameter_or("planner.limit_margin", limit_margin_, 0.3);
    node_->get_parameter_or("planner.limit_weight", limit_weight_, 5.0);
    // > 0: one velocity/acceleration scaling for every planned motion (real robot).
    node_->get_parameter_or("planner.velocity_scaling", velocity_scaling_, 0.0);
    node_->get_parameter_or("observe.pose", observe_pose_, std::string("observe"));
    node_->get_parameter_or("observe.wait", observe_wait_, 6.0);
    node_->get_parameter_or("gripper.guarded_close", guarded_close_, true);
    node_->get_parameter_or("gripper.contact_force", contact_force_, 2.0);
    node_->get_parameter_or("gripper.close_speed", close_speed_, 0.005);
    node_->get_parameter_or("gripper.overtravel", overtravel_, 0.008);
    node_->get_parameter_or("approach.min_distance", approach_min_, 0.16);
    node_->get_parameter_or("approach.max_distance", approach_max_, 0.20);
    node_->get_parameter_or("lift.min_distance", lift_min_, 0.15);
    node_->get_parameter_or("lift.max_distance", lift_max_, 0.18);
    node_->get_parameter_or("max_solutions", max_solutions_, 5);
    node_->get_parameter_or("gazebo_attach_timeout", gazebo_attach_timeout_, 5.0);
    std::vector<double> place_position;
    node_->get_parameter_or(
        "place.position", place_position, std::vector<double>{ 0.30, 0.25, 0.30 });
    if (place_position.size() != 3 || max_solutions_ <= 0)
      throw std::runtime_error("Invalid place.position or max_solutions");
    place_pose_.header.frame_id = world_frame_;
    place_pose_.pose.position.x = place_position[0];
    place_pose_.pose.position.y = place_position[1];
    place_pose_.pose.position.z = place_position[2];
    place_pose_.pose.orientation.x = 1.0;
    place_pose_.pose.orientation.w = 0.0;

    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    plan_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/fr3_demo/plan_pick_task",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          approach_only_ = false;
          plan(response);
        }, rmw_qos_profile_services_default, callback_group_);
    execute_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/fr3_demo/execute_pick_task",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          execute(response);
        }, rmw_qos_profile_services_default, callback_group_);
    // Approach only: the same chain up to the gripper around the object (open,
    // free-space move, Cartesian approach to the grasp pose) - no close, attach,
    // lift, carry or place. For checking reach/approach before a gripper exists.
    approach_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/fr3_demo/plan_approach_task",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          approach_only_ = true;
          plan(response);
        }, rmw_qos_profile_services_default, callback_group_);
    // Execute only up to the pregrasp (end of the free-space move); the next
    // /fr3_demo/execute_pick_task call continues from there.
    pregrasp_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/fr3_demo/execute_to_pregrasp",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          execute(response, true);
        }, rmw_qos_profile_services_default, callback_group_);
    run_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/fr3_demo/run_mtc_pick",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          // Perceive from the observe pose (arm clear of the camera's view),
          // then plan the whole chain, then execute it.
          std::string error;
          if (!moveToObservePose(error))
          {
            response->success = false;
            response->message = "TASK_OBSERVE_FAILED: " + error;
            RCLCPP_ERROR(logger_, "%s", response->message.c_str());
            return;
          }
          approach_only_ = false;
          plan(response);
          if (response->success)
            execute(response);
        }, rmw_qos_profile_services_default, callback_group_);

    rclcpp::SubscriptionOptions state_options;
    state_options.callback_group = callback_group_;
    // The target can also be changed with `ros2 param set`, but MTC creates a
    // second node with this name, so that command may report a failure even
    // when it worked. This topic is the unambiguous way.
    {
      std::string target;
      node_->get_parameter_or("target", target, std::string("green box"));
      target_ = target;
    }
    target_subscription_ = node_->create_subscription<std_msgs::msg::String>(
        "/fr3_demo/pick_target", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(perceived_mutex_);
          target_ = msg->data;
          RCLCPP_INFO(logger_, "Pick target set to '%s'", msg->data.c_str());
        }, state_options);
    param_callback_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
      for (const auto& param : params)
        if (param.get_name() == "target")
        {
          std::lock_guard<std::mutex> lock(perceived_mutex_);
          target_ = param.as_string();
        }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });
    perceived_subscription_ =
        node_->create_subscription<fairino_fr3_msgs::msg::PerceivedObjectArray>(
            "/fr3_demo/perceived_objects", rclcpp::QoS(1),
            [this](fairino_fr3_msgs::msg::PerceivedObjectArray::ConstSharedPtr msg) {
              std::lock_guard<std::mutex> lock(perceived_mutex_);
              perceived_ = *msg;
            }, state_options);

    observe_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/fr3_demo/move_to_observe",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          std::string error;
          response->success = moveToObservePose(error);
          response->message = response->success ? "at observe pose, perception refreshed" : error;
        }, rmw_qos_profile_services_default, callback_group_);

    gazebo_attach_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(
        "/fr3_demo/gazebo_attach", rclcpp::QoS(1).reliable());
    gazebo_detach_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(
        "/fr3_demo/gazebo_detach", rclcpp::QoS(1).reliable());
    gazebo_state_subscription_ = node_->create_subscription<std_msgs::msg::String>(
        "/fr3_demo/gazebo_attachment_state", rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          // DetachableJoint publishes "attached" or "detached".
          gazebo_attached_ = (msg->data == "attached");
          have_gazebo_state_ = true;
        }, state_options);

    // Must exist before the node is spun: action clients created later from a
    // service callback never receive their goal responses.
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        node_, arm_group_);
    hand_move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        node_, hand_group_);
    finger_forces_subscription_ =
        node_->create_subscription<fairino_fr3_msgs::msg::FingerForces>(
            "/fr3_demo/gripper/finger_forces", rclcpp::QoS(10),
            [this](fairino_fr3_msgs::msg::FingerForces::ConstSharedPtr msg) {
              std::lock_guard<std::mutex> lock(forces_mutex_);
              finger_forces_ = *msg;
              forces_received_ = std::chrono::steady_clock::now();
            }, state_options);

    RCLCPP_INFO(logger_, "MTC plan+execute ready: /fr3_demo/run_mtc_pick");
    RCLCPP_INFO(logger_, "MTC preflight ready: /fr3_demo/plan_pick_task");
    RCLCPP_INFO(logger_, "MTC approach-only preflight ready: /fr3_demo/plan_approach_task");
    RCLCPP_INFO(logger_, "MTC execution ready: /fr3_demo/execute_pick_task");
  }

private:
  void buildTask()
  {
    task_ = std::make_unique<mtc::Task>();
    task_->stages()->setName(approach_only_ ? "FR3 approach-only preflight" :
                                              "FR3 complete pick preflight");
    task_->loadRobotModel(node_);

    auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    pipeline->setProperty("goal_joint_tolerance", 1e-4);
    // Free-space moves: Pilz PTP first (straight line in joint space, shortest
    // and repeatable; IK seeded from the start state so no wrist flips), OMPL
    // only if PTP collides.
    mtc::solvers::PlannerInterfacePtr free_space = pipeline;
    ptp_planner_.reset();
    if (ptp_first_)
    {
      auto ptp = std::make_shared<mtc::solvers::PipelinePlanner>(
          node_, "pilz_industrial_motion_planner");
      ptp->setPlannerId("PTP");
      ptp->setProperty("goal_joint_tolerance", 1e-4);
      ptp_planner_ = ptp;
      free_space = std::make_shared<mtc::solvers::MultiPlanner>(
          mtc::solvers::MultiPlanner{ ptp, pipeline });
    }
    // Rank whole-task solutions by how far the joints travel, plus a penalty
    // for passing close to a joint limit: per joint, w * max(0, (m0 - m)/m0)^2
    // with m the smallest distance to its limit along the segment. Far from
    // the limits (m >= m0) it costs nothing; near them a slightly longer path
    // wins, but a near-limit solution is still used if it is the only one.
    const double m0 = limit_margin_;
    const double w = limit_weight_;
    auto joint_path_length = std::make_shared<mtc::LambdaCostTerm>(
        [m0, w](const mtc::SubTrajectory& s, std::string& comment) {
          const double path = mtc::cost::PathLength()(s, comment);
          const auto& trajectory = s.trajectory();
          if (!trajectory || trajectory->empty() || m0 <= 0.0)
            return path;
          const auto* group = trajectory->getGroup();
          const auto& joints = group ? group->getActiveJointModels() :
                                       trajectory->getRobotModel()->getActiveJointModels();
          double penalty = 0.0;
          for (const auto* joint : joints)
          {
            const auto& bounds = joint->getVariableBounds()[0];
            if (!bounds.position_bounded_)
              continue;
            const int index = joint->getFirstVariableIndex();
            double margin = std::numeric_limits<double>::infinity();
            for (std::size_t k = 0; k < trajectory->getWayPointCount(); ++k)
            {
              const double q = trajectory->getWayPoint(k).getVariablePosition(index);
              margin = std::min(margin, std::min(q - bounds.min_position_, bounds.max_position_ - q));
            }
            const double closeness = std::max(0.0, (m0 - margin) / m0);
            penalty += w * closeness * closeness;
          }
          if (penalty > 0.0)
            comment += "joint-limit penalty " + std::to_string(penalty) + " ";
          return path + penalty;
        });
    auto hand_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    hand_planner->setMaxVelocityScalingFactor(0.20);
    hand_planner->setMaxAccelerationScalingFactor(0.15);
    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian->setStepSize(0.005);
    cartesian->setMaxVelocityScalingFactor(0.12);
    cartesian->setMaxAccelerationScalingFactor(0.10);
    if (velocity_scaling_ > 0.0)
    {
      for (const auto& planner : std::vector<mtc::solvers::PlannerInterfacePtr>{
               pipeline, free_space, hand_planner, cartesian })
      {
        planner->setMaxVelocityScalingFactor(velocity_scaling_);
        planner->setMaxAccelerationScalingFactor(velocity_scaling_);
      }
      if (ptp_planner_)
      {
        ptp_planner_->setMaxVelocityScalingFactor(velocity_scaling_);
        ptp_planner_->setMaxAccelerationScalingFactor(velocity_scaling_);
      }
      RCLCPP_INFO(logger_, "All motions planned at %.0f%% velocity/acceleration",
                  velocity_scaling_ * 100.0);
    }

    task_->setProperty("group", arm_group_);
    task_->setProperty("eef", eef_);
    task_->setProperty("hand", hand_group_);
    task_->setProperty("hand_grasping_frame", tcp_link_);
    task_->setProperty("ik_frame", tcp_link_);

    {
      auto current = std::make_unique<mtc::stages::CurrentState>("current state");
      auto filter = std::make_unique<mtc::stages::PredicateFilter>(
          "verify object is available", std::move(current));
      filter->setPredicate([id = object_id_](const mtc::SolutionBase& solution,
                                             std::string& comment) {
        const auto scene = solution.start()->scene();
        if (scene->getCurrentState().hasAttachedBody(id))
        {
          comment = "object '" + id + "' is already attached";
          return false;
        }
        if (!scene->getWorld()->hasObject(id))
        {
          comment = "object '" + id + "' is missing from Planning Scene";
          return false;
        }
        return true;
      });
      task_->add(std::move(filter));
    }

    // The pedestal is physically bolted to / resting on the support surface.
    // Permit only that fixed mounting contact.  Moving arm links are still
    // collision checked against work_table.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow fixed robot mount contact with table");
      stage->allowCollisions({ mount_link_ }, { support_id_ }, true);
      task_->add(std::move(stage));
    }

    // This is part of MTC's private planning scene, not a change to the live
    // MoveIt scene.  Put it before every downstream stage so GenerateGraspPose,
    // ComputeIK and the backward Cartesian approach all inherit the same ACM.
    // Only the gripper/object pairs are permitted; arm/table checks stay active.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow gripper contact with object during pick planning");
      if (touch_links_.empty())
        throw std::runtime_error("touch_links must not be empty");
      stage->allowCollisions(object_id_, touch_links_, true);
      task_->add(std::move(stage));
    }

    mtc::Stage* open_stage = nullptr;
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", hand_planner);
      stage->setGroup(hand_group_);
      stage->setGoal("open");
      open_stage = stage.get();
      task_->add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::Connect>(
          "connect current state to pregrasp",
          mtc::stages::Connect::GroupPlannerVector{ { arm_group_, free_space } });
      stage->setCostTerm(joint_path_length);
      stage->setTimeout(8.0);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      task_->add(std::move(stage));
    }

    mtc::Stage* pick_stage = nullptr;
    {
      auto pick = std::make_unique<mtc::SerialContainer>("pick sequence");
      task_->properties().exposeTo(
          pick->properties(), { "eef", "hand", "group", "ik_frame" });
      pick->properties().configureInitFrom(
          mtc::Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
            "cartesian approach", cartesian);
        stage->setCostTerm(joint_path_length);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->properties().set("link", tcp_link_);
        stage->properties().set("marker_ns", "mtc_approach");
        stage->setMinMaxDistance(approach_min_, approach_max_);
        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = tcp_link_;
        direction.vector.z = 1.0;
        stage->setDirection(direction);
        pick->insert(std::move(stage));
      }

      {
        std::unique_ptr<mtc::stages::ComputeIK> ik;
        if (!external_tcp_.empty())
        {
          auto generator = std::make_unique<mtc::stages::GeneratePose>("external camera grasp");
          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = world_frame_;
          pose.pose.position.x = external_tcp_[0];
          pose.pose.position.y = external_tcp_[1];
          pose.pose.position.z = external_tcp_[2];
          pose.pose.orientation.x = external_tcp_[3];
          pose.pose.orientation.y = external_tcp_[4];
          pose.pose.orientation.z = external_tcp_[5];
          pose.pose.orientation.w = external_tcp_[6];
          generator->setPose(pose);
          generator->setMonitoredStage(open_stage);
          ik = std::make_unique<mtc::stages::ComputeIK>("external grasp collision-aware IK", std::move(generator));
          ik->setMaxIKSolutions(16);
          ik->setMinSolutionDistance(0.2);
          ik->setIKFrame(tcp_link_);
        }
        else
        {
        auto generator = std::make_unique<mtc::stages::GenerateGraspPose>(
            "generate 360-degree top grasps");
        generator->properties().configureInitFrom(mtc::Stage::PARENT);
        generator->properties().set("marker_ns", "mtc_grasp_candidates");
        generator->setPreGraspPose("open");
        generator->setObject(object_id_);
        // Box: jaws across the short side only (0 / 180 deg). Cylinder: any angle.
        generator->setAngleDelta(target_is_box_ ? M_PI : cylinder_angle_delta_);
        generator->setMonitoredStage(open_stage);

        ik = std::make_unique<mtc::stages::ComputeIK>(
            "compute collision-aware grasp IK", std::move(generator));
        ik->setMaxIKSolutions(16);
        ik->setMinSolutionDistance(0.2);
        // Gripper/object contact is allowed in the upstream task scene, while
        // collisions with the table and the rest of the robot remain checked.
        ik->setIgnoreCollisions(false);
        // The generated target is the object's center.  Define that target as
        // being below the top-down TCP by grasp_tcp_clearance_, instead of
        // forcing the gripper body into the table at the object's mid-height.
        Eigen::Isometry3d object_from_tcp = Eigen::Isometry3d::Identity();
        // Top-down: TCP z points down. The jaws close along TCP x; perception
        // puts a box's x along its long side, so turn 90 deg to grip its y side.
        object_from_tcp.linear() =
            (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(target_is_box_ ? M_PI / 2.0 : 0.0, Eigen::Vector3d::UnitZ()))
                .toRotationMatrix();
        object_from_tcp.translation().z() = grasp_tcp_clearance_;
        ik->setIKFrame(object_from_tcp, tcp_link_);
        }
        ik->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        ik->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        pick->insert(std::move(ik));
      }
      if (approach_only_)
      {
        task_->add(std::move(pick));
        task_->init();
        return;
      }

      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", hand_planner);
        stage->setGroup(hand_group_);
        // Close only to the measured object width; "closed" (10 mm) would squeeze
        // the object over.
        stage->setGoal(graspJointGoal());
        pick->insert(std::move(stage));
      }
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "allow object support contact before attach");
        stage->allowCollisions({ object_id_ }, { support_id_ }, true);
        pick->insert(std::move(stage));
      }
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "attach object logically");
        stage->attachObject(object_id_, tcp_link_);
        pick->insert(std::move(stage));
      }
      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("cartesian lift", cartesian);
        stage->setCostTerm(joint_path_length);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setIKFrame(tcp_link_);
        stage->setMinMaxDistance(lift_min_, lift_max_);
        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = world_frame_;
        direction.vector.z = 1.0;
        stage->setDirection(direction);
        pick->insert(std::move(stage));
      }
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "restore object-table collision");
        stage->allowCollisions({ object_id_ }, { support_id_ }, false);
        pick->insert(std::move(stage));
      }
      pick_stage = pick.get();
      task_->add(std::move(pick));
    }

    // Carry: several IK solutions for the place pose, each costed by its
    // distance from the post-lift state, joined by a free-space move. A plain
    // MoveTo(pose) let the planner pick any IK branch (e.g. j1 nearly a full
    // turn, j5 half a turn) for a target only ~40 cm away.
    {
      auto stage = std::make_unique<mtc::stages::Connect>(
          "carry attached object to place prepose",
          mtc::stages::Connect::GroupPlannerVector{ { arm_group_, free_space } });
      stage->setTimeout(8.0);
      stage->setCostTerm(joint_path_length);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      task_->add(std::move(stage));
    }
    {
      // The placed object may face any way about the vertical, so offer the
      // place TCP pose at place_yaw_steps headings; with a single fixed heading
      // the wrist had to flip half a turn whenever the grasp yaw disagreed.
      auto candidates = std::make_unique<mtc::Alternatives>("place pose candidates");
      Eigen::Quaterniond nominal(place_pose_.pose.orientation.w, place_pose_.pose.orientation.x,
                                 place_pose_.pose.orientation.y, place_pose_.pose.orientation.z);
      for (int i = 0; i < place_yaw_steps_; ++i)
      {
        const double yaw = 2.0 * M_PI * i / place_yaw_steps_;
        const Eigen::Quaterniond turned =
            Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) * nominal;
        geometry_msgs::msg::PoseStamped target = place_pose_;
        target.header.stamp = node_->now();
        target.pose.orientation.w = turned.w();
        target.pose.orientation.x = turned.x();
        target.pose.orientation.y = turned.y();
        target.pose.orientation.z = turned.z();
        auto pose = std::make_unique<mtc::stages::GeneratePose>(
            "place pose " + std::to_string(static_cast<int>(std::round(yaw * 180.0 / M_PI))) + " deg");
        pose->setPose(target);
        pose->setMonitoredStage(pick_stage);
        auto ik = std::make_unique<mtc::stages::ComputeIK>("place IK", std::move(pose));
        ik->setMaxIKSolutions(4);
        ik->setMinSolutionDistance(0.2);
        ik->setIKFrame(tcp_link_);
        ik->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        ik->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        candidates->insert(std::move(ik));
      }
      task_->properties().exposeTo(candidates->properties(), { "eef", "group" });
      candidates->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      task_->add(std::move(candidates));
    }

    if (release_into_box_)
    {
      auto vertical = [&](const std::string& name, double z, double distance) {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(name, cartesian);
        stage->setGroup(arm_group_);
        stage->setIKFrame(tcp_link_);
        stage->setMinMaxDistance(distance * 0.95, distance);
        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = world_frame_;
        direction.vector.z = z;
        stage->setDirection(direction);
        task_->add(std::move(stage));
      };
      vertical("lower mouse into open box", -1.0, 0.13);
      auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("release mouse inside box");
      detach->detachObject(object_id_, tcp_link_);
      task_->add(std::move(detach));
      auto open = std::make_unique<mtc::stages::MoveTo>("open jaws inside box", hand_planner);
      open->setGroup(hand_group_);
      open->setGoal("open");
      task_->add(std::move(open));
      vertical("withdraw above box", 1.0, 0.15);
      auto home = std::make_unique<mtc::stages::MoveTo>("return home", free_space);
      home->setGroup(arm_group_);
      home->setGoal("home");
      task_->add(std::move(home));
    }
    task_->init();
  }

  void plan(const std_srvs::srv::Trigger::Response::SharedPtr& response)
  {
    std::unique_lock<std::mutex> lock(task_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
      response->success = false;
      response->message = "TASK_BUSY: another plan or execution is running";
      return;
    }
    try
    {
      solution_ready_ = false;
      RCLCPP_INFO(logger_, "TASK_PREFLIGHT_STARTED: evaluating complete pick chain");
      std::string error;
      if (!resolveTarget(error))
      {
        RCLCPP_ERROR(logger_, "TASK_TARGET_INVALID: %s. Robot was not moved.", error.c_str());
        response->success = false;
        response->message = "TASK_TARGET_INVALID: " + error;
        return;
      }
      // Connect needs the hand at exactly "open" (1e-4); a jaw that settled a
      // fraction of a millimetre away (e.g. -0.00129 vs -0.001) makes every
      // plan fail. Snap the jaw only (the arm is not moved).
      if (!jawsAtOpen() && !openJawsExactly())
        RCLCPP_WARN(logger_, "could not snap the jaws to 'open'; planning anyway");
      buildTask();
      if (!task_->plan(static_cast<std::size_t>(max_solutions_)))
      {
        std::ostringstream explanation;
        task_->explainFailure(explanation);
        const std::string details = explanation.str().empty() ?
            "MTC found no connected solution" : explanation.str();
        RCLCPP_ERROR(logger_, "TASK_PLANNING_FAILED\n%s\nRobot was not moved.", details.c_str());
        response->success = false;
        response->message = "TASK_PLANNING_FAILED: " + details;
        return;
      }
      task_->introspection().publishSolution(*task_->solutions().front());
      logPathMetrics(*task_->solutions().front());
      std::string grasp_error;
      if (approach_only_)
        RCLCPP_INFO(logger_, "Approach-only plan: ends with the open gripper at the grasp pose.");
      else if (!checkGraspAlignment(*task_->solutions().front(), grasp_error))
      {
        RCLCPP_ERROR(logger_, "TASK_GRASP_CHECK_FAILED: %s. Robot was not moved.",
                     grasp_error.c_str());
        response->success = false;
        response->message = "TASK_GRASP_CHECK_FAILED: " + grasp_error;
        return;
      }
      solution_ready_ = true;
      resume_index_ = 0;
      planned_approach_only_ = approach_only_.load();
      const double cost = task_->solutions().front()->cost();
      RCLCPP_INFO(
          logger_, "TASK_PLANNING_SUCCESS: %zu solution(s), selected cost %.4f. Robot was not moved.",
          task_->solutions().size(), cost);
      response->success = true;
      response->message = std::string(approach_only_ ?
          "TASK_PLANNING_SUCCESS: approach chain is feasible; cost=" :
          "TASK_PLANNING_SUCCESS: complete chain is feasible; cost=") + std::to_string(cost);
    }
    catch (const mtc::InitStageException& error)
    {
      std::ostringstream message;
      message << error;
      RCLCPP_ERROR(logger_, "TASK_INITIALIZATION_FAILED\n%s", message.str().c_str());
      response->success = false;
      response->message = "TASK_INITIALIZATION_FAILED: " + message.str();
    }
    catch (const std::exception& error)
    {
      RCLCPP_ERROR(logger_, "TASK_PLANNING_EXCEPTION: %s", error.what());
      response->success = false;
      response->message = std::string("TASK_PLANNING_EXCEPTION: ") + error.what();
    }
  }

  void execute(const std_srvs::srv::Trigger::Response::SharedPtr& response,
               bool stop_at_pregrasp = false)
  {
    std::unique_lock<std::mutex> lock(task_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
      response->success = false;
      response->message = "TASK_BUSY: another plan or execution is running";
      return;
    }
    if (!solution_ready_ || !task_ || task_->solutions().empty())
    {
      response->success = false;
      response->message =
          "NO_PLANNED_SOLUTION: call /fr3_demo/plan_pick_task successfully first";
      RCLCPP_ERROR(logger_, "%s", response->message.c_str());
      return;
    }

    RCLCPP_WARN(
        logger_, "TASK_EXECUTION_STARTED: robot controllers will now move the simulated robot");
    // A solution starts from the state at planning time and must never be
    // executed twice. Require a fresh complete-chain preflight for every run;
    // only a pause at the pregrasp keeps it, to be continued from there.
    solution_ready_ = false;
    std::string error;
    std::size_t paused_at = 0;
    if (!executeSolution(*task_->solutions().front(), error, resume_index_,
                         stop_at_pregrasp ? &paused_at : nullptr))
    {
      resume_index_ = 0;
      response->success = false;
      response->message = "TASK_EXECUTION_FAILED: " + error + "; plan again before retrying";
      RCLCPP_ERROR(logger_, "%s", response->message.c_str());
      return;
    }
    if (paused_at > 0)
    {
      solution_ready_ = true;
      resume_index_ = paused_at;
      response->success = true;
      response->message = "TASK_PAUSED_AT_PREGRASP: check the arm, then call "
                          "/fr3_demo/execute_pick_task to continue";
      RCLCPP_WARN(logger_, "%s", response->message.c_str());
      return;
    }
    resume_index_ = 0;
    response->success = true;
    response->message = planned_approach_only_ ?
        "TASK_EXECUTION_SUCCESS: approach completed, gripper open at the grasp pose" :
        "TASK_EXECUTION_SUCCESS: planned pick-and-carry trajectory completed";
    RCLCPP_INFO(logger_, "%s", response->message.c_str());
  }

  // Joint travel per segment (sum of |dq| along the trajectory) and the closest
  // approach to a joint limit, to judge how direct the planned motion is.
  void logPathMetrics(const mtc::SolutionBase& solution)
  {
    moveit_task_constructor_msgs::msg::Solution msg;
    solution.toMsg(msg, &task_->introspection());
    const auto& model = *task_->getRobotModel();
    double total = 0.0;
    for (std::size_t i = 0; i < msg.sub_trajectory.size(); ++i)
    {
      const auto& joints = msg.sub_trajectory[i].trajectory.joint_trajectory;
      if (joints.points.size() < 2)
        continue;
      std::vector<double> travel(joints.joint_names.size(), 0.0);
      double margin = std::numeric_limits<double>::infinity();
      std::string margin_joint;
      for (std::size_t k = 0; k < joints.points.size(); ++k)
        for (std::size_t j = 0; j < joints.joint_names.size(); ++j)
        {
          const double q = joints.points[k].positions[j];
          if (k > 0)
            travel[j] += std::abs(q - joints.points[k - 1].positions[j]);
          const auto& bounds = model.getJointModel(joints.joint_names[j])->getVariableBounds()[0];
          if (bounds.position_bounded_ && std::min(q - bounds.min_position_, bounds.max_position_ - q) < margin)
          {
            margin = std::min(q - bounds.min_position_, bounds.max_position_ - q);
            margin_joint = joints.joint_names[j];
          }
        }
      std::ostringstream per_joint;
      double sum = 0.0;
      for (std::size_t j = 0; j < travel.size(); ++j)
      {
        per_joint << (j ? " " : "") << joints.joint_names[j] << "=" << std::fixed
                  << std::setprecision(2) << travel[j];
        sum += travel[j];
      }
      if (joints.joint_names.size() > 2)
        total += sum;
      RCLCPP_INFO(logger_, "PATH segment %zu: %.2f rad [%s], closest limit %.2f rad (%s), %.1f s",
                  i + 1, sum, per_joint.str().c_str(), margin, margin_joint.c_str(),
                  rclcpp::Duration(joints.points.back().time_from_start).seconds());
      std::ostringstream ends;
      ends << std::fixed << std::setprecision(2) << "start [";
      for (double q : joints.points.front().positions)
        ends << " " << q;
      ends << " ] end [";
      for (double q : joints.points.back().positions)
        ends << " " << q;
      ends << " ]";
      RCLCPP_INFO(logger_, "PATH segment %zu joints: %s", i + 1, ends.str().c_str());
    }
    RCLCPP_INFO(logger_, "PATH total arm joint travel: %.2f rad", total);
  }

  // Independent check of the planned grasp: at the state where the object gets
  // attached, the jaw axis (TCP x) must lie across the box's short side (its y
  // axis) and the TCP must be above the object's centre.
  bool checkGraspAlignment(const mtc::SolutionBase& solution, std::string& error)
  {
    moveit_task_constructor_msgs::msg::Solution msg;
    solution.toMsg(msg, &task_->introspection());
    // Robot state at the attach: the solution's start state with the end point
    // of every trajectory up to the attach applied (arm and hand separately).
    moveit::core::RobotState state(solution.start()->scene()->getCurrentState());
    bool reached_attach = false;
    for (const auto& sub : msg.sub_trajectory)
    {
      const auto& attached = sub.scene_diff.robot_state.attached_collision_objects;
      if (std::any_of(attached.begin(), attached.end(),
                      [this](const auto& object) { return object.object.id == object_id_; }))
      {
        reached_attach = true;
        break;
      }
      const auto& joints = sub.trajectory.joint_trajectory;
      if (!joints.points.empty())
        state.setVariablePositions(joints.joint_names, joints.points.back().positions);
    }
    if (!reached_attach)
    {
      error = "the solution never attaches the object";
      return false;
    }
    state.update();
    const Eigen::Isometry3d tcp = state.getGlobalLinkTransform(tcp_link_);
    const Eigen::Isometry3d object =
        solution.start()->scene()->getFrameTransform(object_id_);

    const Eigen::Vector3d offset = object.inverse() * tcp.translation();
    Eigen::Vector2d jaw = tcp.linear().col(0).head<2>();
    Eigen::Vector2d across = object.linear().col(target_is_box_ ? 1 : 0).head<2>();
    const double angle = std::acos(std::min(
        1.0, std::abs(jaw.normalized().dot(across.normalized())))) * 180.0 / M_PI;
    RCLCPP_INFO(logger_,
                "GRASP CHECK %s: TCP at object (%.1f, %.1f, %.1f) mm, jaw axis %.1f deg "
                "from the object's %s",
                object_id_.c_str(), offset.x() * 1000.0, offset.y() * 1000.0,
                offset.z() * 1000.0, angle, target_is_box_ ? "short side" : "x axis");
    if (target_is_box_ && angle > 10.0)
    {
      error = "planned jaws are " + std::to_string(angle) +
          " deg off the object's short side";
      return false;
    }
    return true;
  }

  // Runs the pre-planned solution segment by segment instead of MTC's
  // one-shot execute, so execution can stop at the logical attach and wait
  // for Gazebo to confirm the physical attach before lifting.
  // Runs segments from `first`. With `paused_at`, stops after the first arm
  // free-space segment (the move to the pregrasp) and returns the next index.
  bool executeSolution(const mtc::SolutionBase& solution, std::string& error,
                       std::size_t first = 0, std::size_t* paused_at = nullptr)
  {
    moveit_task_constructor_msgs::msg::Solution msg;
    solution.toMsg(msg, &task_->introspection());
    bool attached = false;
    // An approach-only plan has no close: its only hand motion is the opening.
    const std::size_t close_index =
        planned_approach_only_ ? msg.sub_trajectory.size() : findCloseSegment(msg);

    const auto& arm_joints = move_group_->getActiveJoints();
    for (std::size_t i = first; i < msg.sub_trajectory.size(); ++i)
    {
      const auto& sub = msg.sub_trajectory[i];
      if (guarded_close_ && i == close_index)
      {
        waitUntilSettled();
        if (!guardedClose(error))
          return false;
        if (!applySceneDiff(sub.scene_diff))
        {
          error = "could not apply planning-scene change of segment " + std::to_string(i + 1);
          return false;
        }
        continue;
      }
      // The first segment whose scene change attaches the object is the
      // logical attach stage: weld the box in Gazebo before going further.
      const auto& attached_objects = sub.scene_diff.robot_state.attached_collision_objects;
      const bool attaches_here = !attached &&
          std::any_of(attached_objects.begin(), attached_objects.end(),
                      [this](const auto& object) { return object.object.id == object_id_ && object.object.operation == moveit_msgs::msg::CollisionObject::ADD; });
      const bool detaches_here = attached && std::any_of(attached_objects.begin(), attached_objects.end(),
          [this](const auto& object) { return object.object.id == object_id_ && object.object.operation == moveit_msgs::msg::CollisionObject::REMOVE; });
      if (detaches_here && !commandGazeboAttachment(false))
      {
        error = "Gazebo did not confirm release inside the box";
        return false;
      }
      attached = attached || attaches_here;
      if (attaches_here && !commandGazeboAttachment(true))
      {
        error = "Gazebo did not confirm the physical attach; the object was not lifted";
        return false;
      }
      // A single point means the stage needed no motion (e.g. already open).
      if (sub.trajectory.joint_trajectory.points.size() > 1)
      {
        waitUntilSettled();
        RCLCPP_INFO(logger_, "Executing segment %zu/%zu (%zu points)", i + 1,
                    msg.sub_trajectory.size(), sub.trajectory.joint_trajectory.points.size());
        const auto result = move_group_->execute(sub.trajectory);
        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
          error = "segment " + std::to_string(i + 1) + " failed with MoveIt error code " +
              std::to_string(result.val);
          return false;
        }
      }
      if (!applySceneDiff(sub.scene_diff))
      {
        error = "could not apply planning-scene change of segment " + std::to_string(i + 1);
        return false;
      }
      const auto& names = sub.trajectory.joint_trajectory.joint_names;
      const bool arm_motion = sub.trajectory.joint_trajectory.points.size() > 1 &&
          std::any_of(names.begin(), names.end(), [&arm_joints](const std::string& name) {
            return std::find(arm_joints.begin(), arm_joints.end(), name) != arm_joints.end();
          });
      if (paused_at && arm_motion && i + 1 < msg.sub_trajectory.size())
      {
        *paused_at = i + 1;
        return true;
      }
    }
    return true;
  }

  // Move the arm out of the camera's view and give perception time to refresh
  // (new objects need several confirmations; poses are averaged over ~5 s).
  bool moveToObservePose(std::string& error)
  {
    if (observe_pose_.empty())
      return true;
    move_group_->setStartStateToCurrentState();
    if (!move_group_->setNamedTarget(observe_pose_))
    {
      error = "unknown named pose '" + observe_pose_ + "'";
      return false;
    }
    if (move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS)
    {
      error = "could not move to '" + observe_pose_ + "'";
      return false;
    }
    // Put the jaws exactly at "open": the planner's Connect stage requires the
    // hand to match the planned "open" state to 1e-4, but the jaw controller
    // holds wherever the jaw was when it started (e.g. -0.00128 vs -0.001).
    if (!openJawsExactly())
    {
      error = "could not open the jaws";
      return false;
    }
    RCLCPP_INFO(logger_, "At '%s'; waiting %.1f s for perception to refresh",
                observe_pose_.c_str(), observe_wait_);
    std::this_thread::sleep_for(std::chrono::duration<double>(observe_wait_));
    return true;
  }

  // True when the hand is within 1e-4 of "open"; small offsets (< 3 mm) are
  // drift worth snapping, larger ones mean the jaws are really closed.
  bool jawsAtOpen()
  {
    const auto* group = hand_move_group_->getRobotModel()->getJointModelGroup(hand_group_);
    std::map<std::string, double> open;
    if (!group->getVariableDefaultPositions("open", open))
      return true;
    const auto names = hand_move_group_->getActiveJoints();
    const auto current = hand_move_group_->getCurrentJointValues();
    for (std::size_t i = 0; i < names.size() && i < current.size(); ++i)
    {
      const double deviation = std::abs(current[i] - open.at(names[i]));
      if (deviation > 1e-4 && deviation < 0.003)
      {
        RCLCPP_INFO(logger_, "jaw %s at %.5f, 'open' is %.5f: snapping", names[i].c_str(),
                    current[i], open.at(names[i]));
        return false;
      }
    }
    return true;
  }

  bool openJawsExactly()
  {
    const auto* group = task_ ? task_->getRobotModel()->getJointModelGroup(hand_group_) :
                                hand_move_group_->getRobotModel()->getJointModelGroup(hand_group_);
    std::map<std::string, double> open;
    if (!group->getVariableDefaultPositions("open", open))
      return false;
    moveit_msgs::msg::RobotTrajectory trajectory;
    auto& joint_trajectory = trajectory.joint_trajectory;
    joint_trajectory.joint_names = hand_move_group_->getActiveJoints();
    trajectory_msgs::msg::JointTrajectoryPoint start, end;
    start.positions = hand_move_group_->getCurrentJointValues();
    for (const auto& name : joint_trajectory.joint_names)
      end.positions.push_back(open.at(name));
    end.time_from_start = rclcpp::Duration::from_seconds(1.0);
    joint_trajectory.points = { start, end };
    return hand_move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  // The jaw-closing segment: the last hand-only motion before the attach.
  std::size_t findCloseSegment(const moveit_task_constructor_msgs::msg::Solution& msg) const
  {
    const auto& hand = hand_move_group_->getActiveJoints();
    std::size_t close = msg.sub_trajectory.size();
    for (std::size_t i = 0; i < msg.sub_trajectory.size(); ++i)
    {
      const auto& sub = msg.sub_trajectory[i];
      const auto& attached = sub.scene_diff.robot_state.attached_collision_objects;
      if (std::any_of(attached.begin(), attached.end(),
                      [this](const auto& object) { return object.object.id == object_id_; }))
        break;
      const auto& names = sub.trajectory.joint_trajectory.joint_names;
      const bool hand_only = !names.empty() &&
          std::all_of(names.begin(), names.end(), [&hand](const std::string& name) {
            return std::find(hand.begin(), hand.end(), name) != hand.end();
          });
      if (hand_only && sub.trajectory.joint_trajectory.points.size() > 1)
        close = i;
    }
    return close;
  }

  // Close the jaws slowly past the measured width and stop as soon as both
  // jaw force sensors (RFP602) report contact_force. The camera width then only
  // has to be good to a few mm; the pads decide when the object is held.
  bool guardedClose(std::string& error)
  {
    std::map<std::string, double> goal = graspJointGoal();
    const auto names = hand_move_group_->getActiveJoints();
    const auto current = hand_move_group_->getCurrentJointValues();
    moveit_msgs::msg::RobotTrajectory trajectory;
    auto& joint_trajectory = trajectory.joint_trajectory;
    joint_trajectory.joint_names = names;
    trajectory_msgs::msg::JointTrajectoryPoint start, end;
    start.positions = current;
    double travel = 0.0;
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      // graspJointGoal() is the measured width (+ squeeze); go overtravel past it.
      const double target = goal.at(names[i]);
      const double past = std::copysign(
          std::min(std::abs(target) + overtravel_ - grasp_squeeze_, gripper_joint_limit_), target);
      end.positions.push_back(past);
      travel = std::max(travel, std::abs(past - current[i]));
    }
    const double duration = std::max(travel / close_speed_, 0.5);
    end.time_from_start = rclcpp::Duration::from_seconds(duration);
    joint_trajectory.points = { start, end };

    RCLCPP_INFO(logger_, "Guarded close: up to %.1f mm per jaw at %.0f mm/s, stop at %.1f N",
                travel * 1000.0, close_speed_ * 1000.0, contact_force_);
    hand_move_group_->asyncExecute(trajectory);
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(duration + 1.0);
    fairino_fr3_msgs::msg::FingerForces forces;
    while (std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(10ms);
      std::chrono::steady_clock::time_point received;
      {
        std::lock_guard<std::mutex> lock(forces_mutex_);
        forces = finger_forces_;
        received = forces_received_;
      }
      if (std::chrono::steady_clock::now() - received > 200ms)
        continue;  // no fresh sensor data
      if (forces.force_clamp_1 >= contact_force_ && forces.force_clamp_2 >= contact_force_)
      {
        hand_move_group_->stop();
        RCLCPP_INFO(logger_, "Grasp detected: pad forces %.1f N / %.1f N",
                    forces.force_clamp_1, forces.force_clamp_2);
        std::this_thread::sleep_for(300ms);
        return true;
      }
    }
    hand_move_group_->stop();
    error = "jaws closed without both pads touching (" + std::to_string(forces.force_clamp_1) +
        " N / " + std::to_string(forces.force_clamp_2) + " N); nothing was lifted";
    return false;
  }

  bool applySceneDiff(moveit_msgs::msg::PlanningScene diff)
  {
    if (!diff.is_diff)
      return true;
    // Joint values come from the real controllers; only keep scene changes such
    // as the attached object and allowed collisions.
    diff.robot_state.joint_state = sensor_msgs::msg::JointState();
    diff.robot_state.multi_dof_joint_state = sensor_msgs::msg::MultiDOFJointState();
    diff.robot_state.is_diff = true;
    if (diff.world.collision_objects.empty() &&
        diff.robot_state.attached_collision_objects.empty() &&
        diff.allowed_collision_matrix.entry_names.empty())
      return true;
    return scene_.applyPlanningScene(diff);
  }

  bool commandGazeboAttachment(bool attach)
  {
    auto publisher = attach ? gazebo_attach_publisher_ : gazebo_detach_publisher_;
    if (publisher->get_subscription_count() == 0)
    {
      RCLCPP_ERROR(logger_, "Gazebo attachment bridge is unavailable");
      return false;
    }
    RCLCPP_INFO(logger_, "Requesting Gazebo physical %s", attach ? "attach" : "detach");
    publisher->publish(std_msgs::msg::Empty());
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(gazebo_attach_timeout_);
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (have_gazebo_state_ && gazebo_attached_ == attach)
        return true;
      std::this_thread::sleep_for(50ms);
    }
    return false;
  }

  // Gazebo tracks trajectories with a lag; wait until the robot stops moving so
  // the next segment starts within MoveIt's allowed_start_tolerance.
  void waitUntilSettled()
  {
    std::vector<double> previous = move_group_->getCurrentJointValues();
    for (int i = 0; i < 30; ++i)
    {
      std::this_thread::sleep_for(100ms);
      const std::vector<double> current = move_group_->getCurrentJointValues();
      double max_change = 0.0;
      for (std::size_t j = 0; j < current.size() && j < previous.size(); ++j)
        max_change = std::max(max_change, std::abs(current[j] - previous[j]));
      if (max_change < 1e-4)
        return;
      previous = current;
    }
    RCLCPP_WARN(logger_, "Robot did not fully settle within 3 s; continuing");
  }

  static std::string lowercase(std::string text)
  {
    std::transform(text.begin(), text.end(), text.begin(), ::tolower);
    return text;
  }

  static std::string describe(const fairino_fr3_msgs::msg::PerceivedObject& object)
  {
    return object.id + " (" + object.color + " " + object.shape +
        (object.graspable ? "" : ", too wide") + ")";
  }

  // Picks the perceived object named by the "target" parameter ("obj_3" or
  // colour/shape words such as "green box") and derives the grasp from its size.
  bool resolveTarget(std::string& error)
  {
    std::string target;
    fairino_fr3_msgs::msg::PerceivedObjectArray perceived;
    {
      std::lock_guard<std::mutex> lock(perceived_mutex_);
      perceived = perceived_;
      target = lowercase(target_);
    }
    if (perceived.objects.empty())
    {
      error = "no perceived objects; is tabletop_perception running?";
      return false;
    }
    std::vector<std::string> words;
    std::istringstream stream(target);
    for (std::string word; stream >> word;)
      words.push_back(word);

    std::vector<const fairino_fr3_msgs::msg::PerceivedObject*> matches;
    std::string seen;
    for (const auto& object : perceived.objects)
    {
      seen += (seen.empty() ? "" : ", ") + describe(object);
      if (object.id == target)
      {
        matches = { &object };
        break;
      }
      const bool all_words_match = !words.empty() &&
          std::all_of(words.begin(), words.end(), [&object](const std::string& word) {
            return word == object.color || word == object.shape;
          });
      if (all_words_match)
        matches.push_back(&object);
    }
    if (matches.empty())
    {
      error = "no object matches '" + target + "'; seen: " + seen;
      return false;
    }
    if (matches.size() > 1)
    {
      error = "'" + target + "' matches " + std::to_string(matches.size()) +
          " objects; use an id. Seen: " + seen;
      return false;
    }
    const auto& object = *matches.front();
    if (!object.graspable)
    {
      error = describe(object) + " is wider (" + std::to_string(object.grasp_width) +
          " m) than the gripper can take";
      return false;
    }

    const bool is_box = object.shape_type == fairino_fr3_msgs::msg::PerceivedObject::SHAPE_BOX;
    const double height = is_box ? object.dimensions[2] : object.dimensions[0];
    // TCP height above the object centre: the object top must stay
    // palm_clearance below the palm, the fingertips table_clearance above the
    // table, and the fingers must overlap the object by min_finger_contact.
    const double above_palm = height / 2.0 - (tcp_to_palm_ - palm_clearance_);
    const double above_table = tcp_to_fingertip_ + table_clearance_ - height / 2.0;
    const double contact_limit = height / 2.0 + tcp_to_fingertip_ - min_finger_contact_;
    const double clearance = std::max(above_palm, above_table);
    if (clearance > contact_limit)
    {
      error = describe(object) + " is too short (" + std::to_string(height) +
          " m) for the fingers to hold it";
      return false;
    }
    // Positive squeeze presses the jaws onto the object; negative leaves a gap.
    const double joint = (jaw_gap_at_zero_ - object.grasp_width) / 2.0 + grasp_squeeze_;
    if (joint < 0.0 || joint > gripper_joint_limit_)
    {
      error = describe(object) + " width is outside the gripper stroke";
      return false;
    }

    object_id_ = object.id;
    target_is_box_ = is_box;
    grasp_tcp_clearance_ = clearance;
    grasp_joint_ = joint;
    RCLCPP_INFO(logger_,
                "TARGET %s: width %.1f mm, height %.1f mm -> TCP %.1f mm above centre, "
                "jaw joints +/-%.4f",
                describe(object).c_str(), object.grasp_width * 1000.0, height * 1000.0,
                clearance * 1000.0, joint);
    return true;
  }

  std::map<std::string, double> graspJointGoal() const
  {
    // One value per actuated hand joint (a single motor here: joint_clamp_2
    // mimics joint_clamp_1); the closing direction comes from the SRDF "closed"
    // state.
    const auto* group = task_->getRobotModel()->getJointModelGroup(hand_group_);
    std::map<std::string, double> closed;
    if (!group->getVariableDefaultPositions("closed", closed))
      throw std::runtime_error("hand group has no 'closed' state in the SRDF");
    std::map<std::string, double> goal;
    for (const auto& name : group->getActiveJointModelNames())
      goal[name] = std::copysign(grasp_joint_, closed.at(name));
    return goal;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::string arm_group_;
  std::string hand_group_;
  std::string eef_;
  std::string tcp_link_;
  std::string object_id_;
  std::string support_id_;
  std::string mount_link_;
  std::vector<std::string> touch_links_;
  std::string world_frame_;
  double cylinder_angle_delta_;
  double grasp_tcp_clearance_{ 0.0 };
  double jaw_gap_at_zero_;
  double gripper_joint_limit_;
  double grasp_squeeze_;
  double tcp_to_palm_;
  double tcp_to_fingertip_;
  double palm_clearance_;
  double table_clearance_;
  double min_finger_contact_;
  bool target_is_box_{ true };
  std::mutex perceived_mutex_;
  std::string target_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  fairino_fr3_msgs::msg::PerceivedObjectArray perceived_;
  rclcpp::Subscription<fairino_fr3_msgs::msg::PerceivedObjectArray>::SharedPtr
      perceived_subscription_;
  double approach_min_;
  double approach_max_;
  double lift_min_;
  double lift_max_;
  int max_solutions_;
  bool release_into_box_ = false;
  std::vector<double> external_tcp_;
  geometry_msgs::msg::PoseStamped place_pose_;
  std::unique_ptr<mtc::Task> task_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
  std::mutex task_mutex_;
  bool solution_ready_{ false };
  // Set by the service that triggered planning; read by buildTask()/plan().
  std::atomic<bool> approach_only_{ false };
  bool planned_approach_only_{ false };
  std::size_t resume_index_{ 0 };  // next segment after a pause at the pregrasp
  double velocity_scaling_{ 0.0 };
  mtc::solvers::PlannerInterfacePtr ptp_planner_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pregrasp_service_;  // kind of the solution held for execute()

  double grasp_joint_{ 0.0 };  // jaw joint stroke for the target, per jaw
  double gazebo_attach_timeout_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> hand_move_group_;
  bool guarded_close_;
  bool ptp_first_;
  int place_yaw_steps_;
  double limit_margin_;
  double limit_weight_;
  std::string observe_pose_;
  double observe_wait_;
  double contact_force_;
  double close_speed_;
  double overtravel_;
  std::mutex forces_mutex_;
  fairino_fr3_msgs::msg::FingerForces finger_forces_;
  std::chrono::steady_clock::time_point forces_received_;
  rclcpp::Subscription<fairino_fr3_msgs::msg::FingerForces>::SharedPtr finger_forces_subscription_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr approach_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr observe_service_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr gazebo_attach_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr gazebo_detach_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gazebo_state_subscription_;
  std::atomic_bool gazebo_attached_{ false };
  std::atomic_bool have_gazebo_state_{ false };
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  // MoveItConfigsBuilder passes robot model, kinematics, OMPL plugin and
  // request-adapter settings as parameter overrides.  Declaring them here is
  // required so PipelinePlanner loads AddTimeOptimalParameterization.
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("mtc_pick_task_planner", options);
  try
  {
    auto planner = std::make_shared<MtcPickTaskPlanner>(node);
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
