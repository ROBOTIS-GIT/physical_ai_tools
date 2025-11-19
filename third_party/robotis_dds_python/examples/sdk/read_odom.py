#!/usr/bin/env python3
"""
DDS Example - Read robot odometry
"""
from robotis_dds_python.robotis_dds_sdk import RobotisDDSSDK
import time

def main():
    rds = RobotisDDSSDK(domain_id=30)
    print("📍 Reading odometry data... (Ctrl+C to exit)")

    try:
        while True:
            odom = rds.get_odometry()
            if odom:
                print(f"X={odom['x']:.2f}, Y={odom['y']:.2f}, θ={odom['theta']:.2f}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    rds.close()

if __name__ == "__main__":
    main()
