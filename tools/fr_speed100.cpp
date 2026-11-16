// Read FR3 state, then set global speed 100% (mode unchanged). Sends no motion.
#include <cstdio>
#include <thread>
#include <chrono>
#include "robot.h"

static void print_state(FRRobot &robot, const char *tag) {
    ROBOT_STATE_PKG pkg{};
    robot.GetRobotRealTimeState(&pkg);
    uint8_t drag = 0;
    robot.IsInDragTeach(&drag);
    printf("%s: robot_mode=%d (0=tu dong,1=tay) robot_state=%d enable=%d loi=%d/%d estop=%d drag=%d\n", tag,
           pkg.robot_mode, pkg.robot_state, pkg.rbtEnableState, pkg.main_code, pkg.sub_code, pkg.EmergencyStop, drag);
    fflush(stdout);
}

int main() {
    FRRobot robot;
    if (robot.RPC("192.168.58.2") != 0) { printf("RPC connect failed\n"); return 1; }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    print_state(robot, "truoc");
    printf("SetSpeed(100) rc=%d\n", robot.SetSpeed(100));
    fflush(stdout);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    print_state(robot, "sau");
    robot.CloseRPC();
    return 0;
}
