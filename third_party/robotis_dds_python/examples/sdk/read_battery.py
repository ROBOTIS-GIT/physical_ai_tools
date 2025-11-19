#!/usr/bin/env python3
"""
DDS Example - Read battery state
"""
from robotis_dds_python.robotis_dds_sdk import RobotisDDSSDK
import time

def main():
    rds = RobotisDDSSDK(domain_id=30)
    print("🔋 Reading battery state... (Ctrl+C to exit)")

    try:
        while True:
            battery = rds.get_battery_state()
            if battery:
                print(f"Voltage: {battery['voltage']:.2f} V | {battery['percentage']*100:.1f}%")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    rds.close()

if __name__ == "__main__":
    main()
