#!/usr/bin/env python3
"""
DDS Example - Read joint states
"""
from robotis_dds_python.robotis_dds_sdk import RobotisDDSSDK
import time

def main():
    rds = RobotisDDSSDK(domain_id=30)
    print("🤖 Reading joint states... (Ctrl+C to exit)")

    try:
        while True:
            joints = rds.get_joint_state()
            if joints:
                print("Joint States:", joints)
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    rds.close()

if __name__ == "__main__":
    main()
