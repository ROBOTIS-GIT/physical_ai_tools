import tqdm

from lerobot.common.robot_devices.motors.configs import DynamixelMotorsBusConfig
from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
from lerobot.common.robot_devices.robots.configs import OMXRobotConfig
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot

# Define motor configurations
leader_config = DynamixelMotorsBusConfig(
    port="/dev/ttyACM0",
    motors={
        "shoulder_pan": (1, "xl330-m288"),
        "shoulder_lift": (2, "xl330-m288"),
        "elbow_flex": (3, "xl330-m288"),
        "wrist_flex": (4, "xl330-m288"),
        "gripper": (5, "xl330-m288"),
    },
)

follower_config = DynamixelMotorsBusConfig(
    port="/dev/ttyUSB0",
    motors={
        "shoulder_pan": (1, "xm430-w350"),
        "shoulder_lift": (2, "xm430-w350"),
        "elbow_flex": (3, "xm430-w350"),
        "wrist_flex": (4, "xm430-w350"),
        "gripper": (5, "xm430-w350"),
    },
)

# Initialize motor buses
leader_arm = DynamixelMotorsBus(leader_config)
follower_arm = DynamixelMotorsBus(follower_config)

# Configure and initialize the robot
robot_config = OMXRobotConfig(
    leader_arms={"main": leader_config},
    follower_arms={"main": follower_config},
    cameras={},
)
robot = ManipulatorRobot(robot_config)
robot.connect()

# Synchronize follower arm with leader arm
seconds = 20
frequency = 200
total_iterations = seconds * frequency

for _ in tqdm.tqdm(range(total_iterations)):
    leader_pos = robot.leader_arms["main"].read("Present_Position")
    print(leader_pos)
    robot.follower_arms["main"].write("Goal_Position", leader_pos)
