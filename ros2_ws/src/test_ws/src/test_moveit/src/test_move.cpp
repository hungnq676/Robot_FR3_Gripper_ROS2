#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
    // Khởi tạo ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("moveit_multi_point_demo");

    // Tạo MoveGroup (group name phải đúng với robot của bạn)
    moveit::planning_interface::MoveGroupInterface move_group(node, "fairino3_v6_arm");

    // Lấy pose hiện tại của robot (TCP)
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

    // Tạo danh sách waypoint
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Điểm bắt đầu
    waypoints.push_back(start_pose);

    // Điểm 1: đi lên theo trục Z
    geometry_msgs::msg::Pose p1 = start_pose;
    p1.position.z += 0.05;

    // Điểm 2: đi sang Y
    geometry_msgs::msg::Pose p2 = p1;
    p2.position.y += 0.05;

    // Điểm 3: đi sang X
    geometry_msgs::msg::Pose p3 = p2;
    p3.position.x += 0.05;

    // Thêm vào danh sách
    waypoints.push_back(p1);
    waypoints.push_back(p2);
    waypoints.push_back(p3);

    // Tạo trajectory
    moveit_msgs::msg::RobotTrajectory trajectory;

    // Tính đường đi (Cartesian path)
    double fraction = move_group.computeCartesianPath(
        waypoints,
        0.01,   // bước nội suy (m)
        0.0,    // jump threshold
        trajectory
    );

    RCLCPP_INFO(node->get_logger(), "Path achieved: %f", fraction);

    // Thực thi chuyển động
    if (fraction > 0.9)
    {
        move_group.execute(trajectory);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Path planning failed!");
    }

    rclcpp::shutdown();
    return 0;
}
