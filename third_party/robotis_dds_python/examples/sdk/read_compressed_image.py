#!/usr/bin/env python3
"""
DDS Example - Read compressed camera image
"""
from robotis_dds_python.robotis_dds_sdk import RobotisDDSSDK
import cv2

def main():
    rds = RobotisDDSSDK(domain_id=30)
    print("📷 Reading DDS camera image stream... (press 'q' to quit)")

    while True:
        frame = rds.get_rgb_image()
        if frame is not None:
            cv2.imshow("DDS Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rds.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
