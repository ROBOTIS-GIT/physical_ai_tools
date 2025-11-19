#!/usr/bin/env python3
"""
DDS Example - Send velocity commands
Publishes geometry_msgs/Twist messages to /cmd_vel.
"""

from robotis_dds_python.robotis_dds_sdk import RobotisDDSSDK
import time, math


def main():
    rds = RobotisDDSSDK(domain_id=30)
    print("🚗 Sending cmd_vel commands (infinite loop)...")

    try:
        t = 0.0
        while True:
            linear_x = 0.2 * math.sin(t)
            angular_z = 0.5 * math.cos(t)
            rds.send_cmd_vel(linear_x, angular_z)
            print(f"[{time.strftime('%H:%M:%S')}] linear={linear_x:.3f}, angular={angular_z:.3f}")
            t += 0.1
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    rds.close()


if __name__ == "__main__":
    main()
