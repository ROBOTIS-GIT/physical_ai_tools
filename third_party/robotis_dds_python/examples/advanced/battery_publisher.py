#!/usr/bin/env python3
"""
BatteryState Publisher Example (CycloneDDS → ROS 2)

Publishes sensor_msgs/BatteryState messages to the /battery_state topic.

Usage:
    python battery_state_publisher.py
    ros2 topic echo /battery_state sensor_msgs/msg/BatteryState
"""

import time
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode

# === IDL message imports ===
from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import Header_
from robotis_dds_python.robotis_dds_core.idl.builtin_interfaces.msg import Time_
from robotis_dds_python.robotis_dds_core.idl.sensor_msgs.msg import BatteryState_


def main():
    print("=== DDS BatteryState Publisher Example ===")
    print("Publishing to /battery_state")
    print("ROS 2 subscribers can receive these messages with:")
    print("  ros2 topic echo /battery_state sensor_msgs/msg/BatteryState\n")
    print("Press Ctrl+C to stop\n")

    node = DDSNode(
        name="battery_state_publisher",
        domain_id=30,
        network_interface="auto",
        allow_multicast=True,
    )

    pub = node.dds_create_publisher("/battery_state", BatteryState_)

    print("Publisher ready! Publishing battery state messages...\n")

    seq = 0
    try:
        while True:
            now = time.time()
            sec = int(now)
            nanosec = int((now - sec) * 1e9)

            header = Header_(
                stamp=Time_(sec=sec, nanosec=nanosec),
                frame_id="battery_link"
            )

            # --- Dummy Battery Data (for testing) ---
            voltage = 12.6 - (seq * 0.01)       # slowly decreasing voltage
            temperature = 25.0 + (seq % 5) * 0.1
            current = -1.5                      # discharging current
            charge = 4.8 - (seq * 0.01)
            capacity = 5.0
            design_capacity = 5.0
            percentage = max(0.0, (charge / capacity) * 100.0)
            power_supply_status = 2             # DISCHARGING
            power_supply_health = 1             # GOOD
            power_supply_technology = 3         # Li-ion
            present = True
            cell_voltage = [voltage / 3.0] * 3  # assume 3 cells
            cell_temperature = [temperature] * 3
            location = "main_battery"
            serial_number = "RB2025-001"

            msg = BatteryState_(
                header=header,
                voltage=voltage,
                temperature=temperature,
                current=current,
                charge=charge,
                capacity=capacity,
                design_capacity=design_capacity,
                percentage=percentage,
                power_supply_status=power_supply_status,
                power_supply_health=power_supply_health,
                power_supply_technology=power_supply_technology,
                present=present,
                cell_voltage=cell_voltage,
                cell_temperature=cell_temperature,
                location=location,
                serial_number=serial_number,
            )

            pub.publish(msg)
            print(f"[Published] seq={seq} voltage={voltage:.2f}V ({percentage:.1f}%)")

            seq += 1
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopping publisher...")

    finally:
        node.dds_destroy_node()
        print("Publisher stopped.")


if __name__ == "__main__":
    main()
