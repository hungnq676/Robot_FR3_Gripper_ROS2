from fairino import Robot
import time

ROBOT_IP = "192.168.58.2"
GRIPPER_INDEX = 1
TIMEOUT_MS = 10000
POLL_DT = 0.25

robot = Robot.RPC(ROBOT_IP)


def dump(prefix=""):
    try:
        active = robot.GetGripperActivateStatus()
    except Exception as e:
        active = f"ERR:{e}"
    try:
        pos = robot.GetGripperCurPosition()
    except Exception as e:
        pos = f"ERR:{e}"
    try:
        motion = robot.GetGripperMotionDone()
    except Exception as e:
        motion = f"ERR:{e}"
    print(prefix, "active=", active, "pos=", pos, "motion=", motion)


print("=== GRIPPER SAFE TEST: ONLY GRIPPER, NO ROBOT MOTION ===")
print("RESET:", robot.ActGripper(GRIPPER_INDEX, 0))
time.sleep(2)
dump("after reset:")

print("ACTIVATE:", robot.ActGripper(GRIPPER_INDEX, 1))
for i in range(40):
    dump(f"activate[{i:02d}]:")
    time.sleep(POLL_DT)

print("\n=== CLOSE ===")
print(
    "MOVE CLOSE:",
    robot.MoveGripper(
        GRIPPER_INDEX,
        100,
        50,
        50,
        TIMEOUT_MS,
        1,
        0, 0, 0, 0,
    )
)
for i in range(40):
    dump(f"close[{i:02d}]:")
    time.sleep(POLL_DT)

print("\n=== OPEN ===")
print(
    "MOVE OPEN:",
    robot.MoveGripper(
        GRIPPER_INDEX,
        0,
        50,
        50,
        TIMEOUT_MS,
        1,
        0, 0, 0, 0,
    )
)
for i in range(40):
    dump(f"open[{i:02d}]:")
    time.sleep(POLL_DT)

print("\n=== FINAL READBACK ===")
dump("final:")
