#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class GraspPoseGenerator : public rclcpp::Node
{
public:
  GraspPoseGenerator()
  : Node("grasp_pose_generator")
  {
    arm_group_ = declare_parameter<std::string>("arm_group", "arm");
    tcp_link_ = declare_parameter<std::string>("tcp_link", "grasp_frame");
    grasp_offset_ = declare_parameter<std::vector<double>>(
        "grasp.offset", { 0.0, 0.0, 0.0 });
    grasp_orientation_ = declare_parameter<std::vector<double>>(
        "grasp.orientation", { 1.0, 0.0, 0.0, 0.0 });
    approach_direction_ = declare_parameter<std::vector<double>>(
        "approach.direction", { 0.0, 0.0, -1.0 });
    approach_distance_ = declare_parameter<double>("approach.distance", 0.12);
    axis_length_ = declare_parameter<double>("marker.axis_length", 0.08);
    point_scale_ = declare_parameter<double>("marker.point_scale", 0.025);
    ik_timeout_seconds_ = declare_parameter<double>("ik.timeout_seconds", 1.0);
    validateParameters();

    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                                 .reliable()
                                 .transient_local();
    object_subscription_ = create_subscription<moveit_msgs::msg::CollisionObject>(
        "/fr3_demo/object_description", latched_qos,
        [this](const moveit_msgs::msg::CollisionObject::SharedPtr message) {
          receiveObject(*message);
        });
    grasp_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/fr3_demo/grasp_pose", latched_qos);
    pregrasp_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/fr3_demo/pregrasp_pose", latched_qos);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/fr3_demo/grasp_markers", latched_qos);
    grasp_ik_publisher_ = create_publisher<std_msgs::msg::Bool>(
        "/fr3_demo/grasp_ik_valid", latched_qos);
    pregrasp_ik_publisher_ = create_publisher<std_msgs::msg::Bool>(
        "/fr3_demo/pregrasp_ik_valid", latched_qos);
    ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
    ik_retry_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]() { sendPendingIkChecks(); });

    RCLCPP_INFO(get_logger(), "Waiting for /fr3_demo/object_description");
    RCLCPP_INFO(get_logger(), "This node publishes grasp candidates but never moves the robot.");
  }

private:
  void validateParameters()
  {
    if (grasp_offset_.size() != 3 || grasp_orientation_.size() != 4 ||
        approach_direction_.size() != 3)
      throw std::runtime_error(
          "Expected grasp.offset[3], grasp.orientation[4], approach.direction[3]");
    if (approach_distance_ <= 0.0 || axis_length_ <= 0.0 ||
        point_scale_ <= 0.0 || ik_timeout_seconds_ <= 0.0)
      throw std::runtime_error("Distances, marker sizes and IK timeout must be positive");

    normalize(grasp_orientation_);
    normalize(approach_direction_);
  }

  static void normalize(std::vector<double>& values)
  {
    double norm_squared = 0.0;
    for (double value : values)
      norm_squared += value * value;
    const double norm = std::sqrt(norm_squared);
    if (norm < 1e-9)
      throw std::runtime_error("Cannot normalize a zero vector/quaternion");
    for (double& value : values)
      value /= norm;
  }

  void receiveObject(const moveit_msgs::msg::CollisionObject& object)
  {
    // object_description_publisher republishes at 1 Hz. One candidate set per
    // process is sufficient; restart this node after changing object.yaml.
    if (has_candidate_)
      return;

    if (object.primitive_poses.empty())
    {
      RCLCPP_ERROR(get_logger(), "Object '%s' has no primitive pose", object.id.c_str());
      return;
    }

    const auto& object_pose = object.primitive_poses.front();
    geometry_msgs::msg::PoseStamped grasp;
    grasp.header.frame_id = object.header.frame_id;
    grasp.header.stamp = now();
    grasp.pose.position.x = object_pose.position.x + grasp_offset_[0];
    grasp.pose.position.y = object_pose.position.y + grasp_offset_[1];
    grasp.pose.position.z = object_pose.position.z + grasp_offset_[2];
    grasp.pose.orientation.x = grasp_orientation_[0];
    grasp.pose.orientation.y = grasp_orientation_[1];
    grasp.pose.orientation.z = grasp_orientation_[2];
    grasp.pose.orientation.w = grasp_orientation_[3];

    geometry_msgs::msg::PoseStamped pregrasp = grasp;
    pregrasp.pose.position.x -= approach_direction_[0] * approach_distance_;
    pregrasp.pose.position.y -= approach_direction_[1] * approach_distance_;
    pregrasp.pose.position.z -= approach_direction_[2] * approach_distance_;

    grasp_pose_ = grasp;
    pregrasp_pose_ = pregrasp;
    has_candidate_ = true;
    ik_requests_pending_ = true;

    grasp_publisher_->publish(grasp_pose_);
    pregrasp_publisher_->publish(pregrasp_pose_);
    marker_publisher_->publish(makeMarkers());
    sendPendingIkChecks();

    RCLCPP_INFO(
        get_logger(),
        "Generated top grasp at [%.3f, %.3f, %.3f] and pregrasp at [%.3f, %.3f, %.3f] in '%s'",
        grasp_pose_.pose.position.x, grasp_pose_.pose.position.y,
        grasp_pose_.pose.position.z, pregrasp_pose_.pose.position.x,
        pregrasp_pose_.pose.position.y, pregrasp_pose_.pose.position.z,
        grasp_pose_.header.frame_id.c_str());
  }

  visualization_msgs::msg::Marker sphereMarker(
      int id, const geometry_msgs::msg::PoseStamped& pose,
      float red, float green, float blue) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = "fr3_grasp_candidates";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = point_scale_;
    marker.scale.y = point_scale_;
    marker.scale.z = point_scale_;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;
    return marker;
  }

  static geometry_msgs::msg::Point rotatedAxisEnd(
      const geometry_msgs::msg::Pose& pose, const std::array<double, 3>& axis,
      double length)
  {
    const double qx = pose.orientation.x;
    const double qy = pose.orientation.y;
    const double qz = pose.orientation.z;
    const double qw = pose.orientation.w;
    const std::array<double, 3> q{ qx, qy, qz };
    const auto cross = [](const std::array<double, 3>& a, const std::array<double, 3>& b) {
      return std::array<double, 3>{
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
      };
    };
    const auto q_cross_v = cross(q, axis);
    std::array<double, 3> inner{
      q_cross_v[0] + qw * axis[0],
      q_cross_v[1] + qw * axis[1],
      q_cross_v[2] + qw * axis[2]
    };
    const auto q_cross_inner = cross(q, inner);
    geometry_msgs::msg::Point end = pose.position;
    end.x += length * (axis[0] + 2.0 * q_cross_inner[0]);
    end.y += length * (axis[1] + 2.0 * q_cross_inner[1]);
    end.z += length * (axis[2] + 2.0 * q_cross_inner[2]);
    return end;
  }

  visualization_msgs::msg::Marker arrowMarker(
      int id, const geometry_msgs::msg::Point& start,
      const geometry_msgs::msg::Point& end, float red, float green, float blue) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = grasp_pose_.header;
    marker.ns = "fr3_grasp_candidates";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.points = { start, end };
    marker.scale.x = 0.008;
    marker.scale.y = 0.016;
    marker.scale.z = 0.024;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;
    return marker;
  }

  visualization_msgs::msg::MarkerArray makeMarkers() const
  {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(sphereMarker(0, grasp_pose_, 0.1F, 1.0F, 0.1F));
    markers.markers.push_back(sphereMarker(1, pregrasp_pose_, 0.1F, 0.4F, 1.0F));
    markers.markers.push_back(arrowMarker(
        2, pregrasp_pose_.pose.position, grasp_pose_.pose.position, 1.0F, 1.0F, 0.0F));
    const auto& start = grasp_pose_.pose.position;
    markers.markers.push_back(arrowMarker(
        10, start, rotatedAxisEnd(grasp_pose_.pose, { 1.0, 0.0, 0.0 }, axis_length_),
        1.0F, 0.0F, 0.0F));
    markers.markers.push_back(arrowMarker(
        11, start, rotatedAxisEnd(grasp_pose_.pose, { 0.0, 1.0, 0.0 }, axis_length_),
        0.0F, 1.0F, 0.0F));
    markers.markers.push_back(arrowMarker(
        12, start, rotatedAxisEnd(grasp_pose_.pose, { 0.0, 0.0, 1.0 }, axis_length_),
        0.0F, 0.0F, 1.0F));
    return markers;
  }

  void sendPendingIkChecks()
  {
    if (!has_candidate_ || !ik_requests_pending_ || !ik_client_->service_is_ready())
      return;
    ik_requests_pending_ = false;
    requestIk(pregrasp_pose_, true, pregrasp_ik_publisher_, "pregrasp");
    // The grasp pose is inside the object by design, so collision checking is
    // disabled here. Contact permission is handled later during grasp execution.
    requestIk(grasp_pose_, false, grasp_ik_publisher_, "grasp");
  }

  void requestIk(
      const geometry_msgs::msg::PoseStamped& pose, bool avoid_collisions,
      const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr& result_publisher,
      const std::string& label)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = arm_group_;
    request->ik_request.robot_state.is_diff = true;
    request->ik_request.ik_link_name = tcp_link_;
    request->ik_request.pose_stamped = pose;
    request->ik_request.avoid_collisions = avoid_collisions;
    const auto nanoseconds = static_cast<int64_t>(ik_timeout_seconds_ * 1e9);
    request->ik_request.timeout.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    request->ik_request.timeout.nanosec =
        static_cast<uint32_t>(nanoseconds % 1000000000LL);
    ik_client_->async_send_request(
        request,
        [this, result_publisher, label](
            rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future) {
          const int error_code = future.get()->error_code.val;
          const bool valid = error_code ==
              moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
          std_msgs::msg::Bool result;
          result.data = valid;
          result_publisher->publish(result);
          if (valid)
            RCLCPP_INFO(get_logger(), "%s IK: VALID", label.c_str());
          else
            RCLCPP_WARN(
                get_logger(), "%s IK: INVALID (MoveIt error code %d)",
                label.c_str(), error_code);
        });
  }

  std::string arm_group_;
  std::string tcp_link_;
  std::vector<double> grasp_offset_;
  std::vector<double> grasp_orientation_;
  std::vector<double> approach_direction_;
  double approach_distance_;
  double axis_length_;
  double point_scale_;
  double ik_timeout_seconds_;
  bool has_candidate_{ false };
  bool ik_requests_pending_{ false };
  geometry_msgs::msg::PoseStamped grasp_pose_;
  geometry_msgs::msg::PoseStamped pregrasp_pose_;
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr object_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pregrasp_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasp_ik_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pregrasp_ik_publisher_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::TimerBase::SharedPtr ik_retry_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<GraspPoseGenerator>());
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("grasp_pose_generator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
