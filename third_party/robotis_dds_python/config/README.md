# CycloneDDS Configuration Files

This directory contains example configuration files for different deployment scenarios.

## Files

- **`cyclonedds.xml`** - Default configuration for local network communication
- **`cyclonedds_robot.xml`** - Configuration for robot (edge device) connecting to cloud
- **`cyclonedds_cloud.xml`** - Configuration for cloud server receiving robot connections

## Usage

### Option 1: Set environment variable

```bash
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
python3 example/trajectory_publisher.py
```

### Option 2: Set in Python code

```python
import os
os.environ['CYCLONEDDS_URI'] = 'file:///absolute/path/to/cyclonedds.xml'

from robotis_dds_python.tools.dds_node import DDSNode
node = DDSNode(name="my_node")
```

### Option 3: Use inline XML

```python
import os
os.environ['CYCLONEDDS_URI'] = '''
<CycloneDDS>
  <Domain>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.100:7400"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
'''
```

## Robot to Cloud Communication

### On Robot (Edge Device):

1. Edit `cyclonedds_robot.xml` and replace `CLOUD_SERVER_IP` with your cloud server's IP
2. Run your application:

```bash
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds_robot.xml
python3 your_robot_node.py
```

### On Cloud Server:

1. Ensure firewall allows UDP ports 7400-7500
2. Run your application:

```bash
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds_cloud.xml
python3 your_cloud_node.py
```

## Network Requirements

- **Firewall**: Open UDP ports 7400-7500
- **NAT**: Port forwarding may be needed for robots behind NAT
- **Cloud Security Group**: Allow inbound UDP traffic on ports 7400-7500

## Testing

Test connection with the ping/pong example:

```bash
# Terminal 1 (Robot)
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds_robot.xml
python3 example/trajectory_publisher.py

# Terminal 2 (Cloud)
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds_cloud.xml
python3 example/trajectory_subscriber.py
```

For more details, see [docs/architecture.md](../docs/architecture.md#network-configuration-for-remote-communication)
