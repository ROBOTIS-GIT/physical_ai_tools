#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DDS ← ROS2: Subscribe to /camera/image (sensor_msgs/Image)
and print received numpy arrays directly.
"""

import time
from robotis_dds_python.robotis_dds_sdk import RobotisDDSSDK
import numpy as np

def main():
    print("🧩 Robotis DDS SDK — Image Numpy Debug")
    sdk = RobotisDDSSDK(domain_id=30)

    try:
        while True:
            frame = sdk.get_image()

            if isinstance(frame, np.ndarray):
                print("\n📦 Received numpy frame:")
                print(frame)
                print("-" * 60)
                time.sleep(1.0)  # 1 second delay to avoid too fast printing
            else:
                print("⏳ Waiting for /camera/image ...", end="\r")
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n🧩 Interrupted by user.")
    finally:
        sdk.close()
        print("✅ DDS closed.")


if __name__ == "__main__":
    main()
