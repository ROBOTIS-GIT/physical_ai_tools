#!/usr/bin/env python3
"""
DDS Example - Send joint trajectory
"""
from robotis_dds_python.robotis_dds_sdk import RobotisDDSSDK
import time, math

def main():
    rds = RobotisDDSSDK(domain_id=30)
    print("🦾 Sending joint trajectory commands...")

    try:
        t = 0.0
        while True:
            positions = [math.sin(t + i) for i in range(6)]
            rds.send_joint_trajectory(positions)
            print(f"Sent joint positions: {positions}")
            t += 0.1
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    rds.close()

if __name__ == "__main__":
    main()
