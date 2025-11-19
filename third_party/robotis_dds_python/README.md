# robotis_dds_python
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![CycloneDDS](https://img.shields.io/badge/CycloneDDS-0.10.x-green.svg)](https://github.com/eclipse-cyclonedds/cyclonedds)

A Python SDK for publishing and subscribing to topics using CycloneDDS without importing rclpy. This library provides a lightweight alternative to ROS 2 Python API while maintaining compatibility with ROS 2 message types.

## Features

- **Lightweight**: No ROS 2 runtime dependencies
- **Compatible**: Works with standard ROS 2 message types
- **Easy to use**: Simple publisher/subscriber API with ROS 2-like interface
- **Extensible**: Support for custom message types
- **Full featured**: Topics, Services, and Actions support

## Installation

### Prerequisites

- Python 3.8 or higher
- CycloneDDS 0.10.2

### Install from source

1. Before using this package, you need to have CycloneDDS installed on your system. You can build it from source as follows:

```bash
# Install build dependencies
sudo apt update
sudo apt install -y git build-essential cmake libssl-dev

# Clone CycloneDDS source
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
cd cyclonedds
git checkout 0.10.2

# Create a build directory
mkdir build && cd build

# Build and install CycloneDDS
cmake -DCMAKE_INSTALL_PREFIX=$HOME/cyclonedds/install -DBUILD_EXAMPLES=ON ..
cmake --build .
cmake --install .
```
* Note: You can change $HOME/cyclonedds/install to any directory you prefer.

2. After installation, make sure to set the environment variables so that Python can locate the CycloneDDS libraries:
```bash
# For bash
echo 'export CYCLONEDDS_HOME=$HOME/cyclonedds/install' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=$CYCLONEDDS_HOME:$CMAKE_PREFIX_PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PATH=$CYCLONEDDS_HOME/bin:$PATH' >> ~/.bashrc

source ~/.bashrc

# For zsh
echo 'export CYCLONEDDS_HOME=$HOME/cyclonedds/install' >> ~/.zshrc
echo 'export CMAKE_PREFIX_PATH=$CYCLONEDDS_HOME:$CMAKE_PREFIX_PATH' >> ~/.zshrc
echo 'export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH' >> ~/.zshrc
echo 'export PATH=$CYCLONEDDS_HOME/bin:$PATH' >> ~/.zshrc

source ~/.zshrc
```

Finally, install robotis_dds_python:

```bash
# Clone the repository
git clone https://github.com/robotis-git/robotis_dds_python.git

# Install the package
cd robotis_dds_python
pip install -e .
```

## Examples

Check the [`examples/`](examples/) directory for complete working examples:

### Basic Topics
- [`publisher.py`](examples/topic/publisher.py) - Publish String messages to ROS 2
- [`subscriber.py`](examples/topic/subscriber.py) - Subscribe to String messages from ROS 2

### Services (using physical_ai_interfaces)
- [`server.py`](examples/service/server.py) - SetRobotType service server
- [`client.py`](examples/service/client.py) - SetRobotType service client

### Advanced Examples
- [`trajectory_publisher.py`](examples/advanced/trajectory_publisher.py) - Publishing joint trajectory messages
- [`trajectory_subscriber.py`](examples/advanced/trajectory_subscriber.py) - Subscribing to joint trajectory messages
- [`multi_topic.py`](examples/advanced/multi_topic.py) - Managing multiple topics with a single node
- [`bidirectional_bridge.py`](examples/advanced/bidirectional_bridge.py) - Bidirectional ROS 2 ↔ DDS bridge
- [`connect_to_ros2.py`](examples/advanced/connect_to_ros2.py) - Connect to ROS 2 system on different network
- [`network_robot.py`](examples/advanced/network_robot.py) - Robot-side network configuration
- [`network_cloud.py`](examples/advanced/network_cloud.py) - Cloud server configuration
- [`ros2_subscribe_dds.py`](examples/advanced/ros2_subscribe_dds.py) - ROS 2 script to subscribe to DDS topics

## Quick Start

### Using Topics

```python
from robotis_dds_python.tools.dds_node import DDSNode
from robotis_dds_python.idl.std_msgs.msg import String_

# Create a node
node = DDSNode(name="my_node")

# Create a subscriber with callback
def callback(msg):
    print(f"Received: {msg.data}")

sub = node.dds_create_subscription('/topic', String_, callback)

# Create a publisher
pub = node.dds_create_publisher('/cmd_topic', String_)
pub.publish(String_(data="Hello"))

# Spin to keep callbacks active
node.dds_spin()
```

### Using Services

```python
from robotis_dds_python.tools.dds_node import DDSNode
from robotis_dds_python.idl.physical_ai_interfaces.srv import (
    SetRobotType_Request, SetRobotType_Response
)

# Server
node = DDSNode(name="server", domain_id=30)

def handle_request(request):
    # Validate robot type (ROBOTIS products)
    valid_types = ['ai_worker', 'omx', 'omy']
    if request.robot_type in valid_types:
        return SetRobotType_Response(
            success=True, 
            message=f'Robot type set to {request.robot_type}'
        )
    else:
        return SetRobotType_Response(
            success=False, 
            message=f'Invalid robot type'
        )

service = node.dds_create_service('/set_robot_type', 
                                  SetRobotType_Request, 
                                  SetRobotType_Response, 
                                  handle_request)
node.dds_spin()

# Client
node = DDSNode(name="client", domain_id=30)
client = node.dds_create_client('/set_robot_type',
                                SetRobotType_Request,
                                SetRobotType_Response)
request = SetRobotType_Request(robot_type='ai_worker')
response = client.call(request, timeout=5.0)
print(f"Success: {response.success}, Message: {response.message}")
```

## Network Communication

### Connecting to ROS 2 Across Different Networks

DDSNode supports network configuration to communicate with ROS 2 systems on different networks (LAN, WAN, or Internet).

#### Same Network (LAN)

```python
from robotis_dds_python.tools.dds_node import DDSNode

# Connect to ROS 2 system on same local network
node = DDSNode(
    name="dds_client",
    domain_id=0,  # Must match ROS 2's ROS_DOMAIN_ID
    peers=["192.168.1.100:7400"],  # ROS 2 system IP
    allow_multicast=True,  # Enable for LAN
    network_interface="auto"
)
```

#### Different Networks (WAN/Internet)

```python
# Connect to ROS 2 system across different networks
node = DDSNode(
    name="robot_node",
    peers=["cloud.example.com:7400", "52.12.34.56:7400"],
    allow_multicast=False,  # Disable for WAN
    network_interface="auto"
)
```

#### Network Parameters

- **`peers`** (list): IP addresses or hostnames of remote DDS systems
  - Format: `"IP:PORT"` or `"HOSTNAME:PORT"`
  - Default port: 7400
  - Example: `["192.168.50.128:7400", "cloud.server.com:7400"]`

- **`allow_multicast`** (bool): Enable/disable multicast discovery
  - `True`: Use multicast (LAN only)
  - `False`: Peer-to-peer only (required for WAN/Internet)

- **`network_interface`** (str): Network interface to use
  - `"auto"`: Auto-detect (recommended)
  - `"0.0.0.0"`: Listen on all interfaces (server mode)
  - `"192.168.1.100"`: Specific interface IP

- **`port_base`** (int): Base port for DDS discovery
  - Default: 7400 (standard DDS port)

### Example: Connect to Remote ROS 2

```python
from robotis_dds_python.tools.dds_node import DDSNode
from robotis_dds_python.idl.std_msgs.msg import String_

# Create node configured for remote communication
node = DDSNode(
    name="dds_client",
    peers=["192.168.50.128:7400"],  # Remote ROS 2 IP
    allow_multicast=True
)

# Subscribe to ROS 2 topic
def callback(msg):
    print(f"From ROS 2: {msg.data}")

sub = node.dds_create_subscription("/chatter", String_, callback)

# Publish to ROS 2 topic
pub = node.dds_create_publisher("/feedback", String_)
pub.publish(String_(data="Hello from DDS!"))

node.dds_spin()
```

### ROS 2 Side Setup

On the ROS 2 system, no special configuration needed! Just run normally:

```bash
# On ROS 2 system (192.168.50.128)
export ROS_DOMAIN_ID=0

# Subscribe to DDS topics
ros2 topic echo /feedback

# Publish to DDS
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from ROS 2'"
```

### Network Scenarios

| Scenario | Configuration | Notes |
|----------|---------------|-------|
| **Same subnet** | `peers=["192.168.1.100:7400"]`<br>`allow_multicast=True` | Simplest setup |
| **Different subnets** | `peers=["10.0.1.100:7400"]`<br>`allow_multicast=False` | Requires router configuration |
| **Internet/Cloud** | `peers=["cloud.com:7400"]`<br>`allow_multicast=False` | Requires public IP or VPN |
| **Multiple peers** | `peers=["IP1:7400", "IP2:7400"]` | Connect to multiple systems |

### Firewall Configuration

Make sure to open UDP ports on both systems:

```bash
# Ubuntu/Debian
sudo ufw allow 7400:7410/udp

# CentOS/RHEL
sudo firewall-cmd --add-port=7400-7410/udp --permanent
sudo firewall-cmd --reload
```

### Troubleshooting Network Issues

1. **Check connectivity:**
   ```bash
   ping 192.168.50.128
   ```

2. **Verify domain ID:**
   ```bash
   # ROS 2 side
   echo $ROS_DOMAIN_ID  # Should be 0 (or match your setting)
   ```

3. **List available topics:**
   ```bash
   ros2 topic list  # Should show DDS topics
   ```

4. **Check if messages are publishing:**
   ```bash
   ros2 topic hz /feedback
   ```

## Documentation

For detailed guides, see the [docs](docs/) directory:

- **[Topics Guide](docs/guides/topics.md)** - Publishing and subscribing to messages
- **[Services Guide](docs/guides/services.md)** - Service servers and clients
- **[Networking Guide](docs/guides/networking.md)** - Network configuration (LAN, WAN, Internet, VPN)
- **[Architecture](docs/architecture.md)** - System design and internals

## Creating Custom Messages

To add support for new ROS 2 message types:

1. Navigate to the IDL directory:
   ```bash
   cd robotis_dds_python/robotis_dds_python/idl
   ```

2. Generate Python bindings from IDL:
   ```bash
   idlc -l py -I /opt/ros/jazzy/share/ /opt/ros/jazzy/share/{your_package}/msg/{YourMessage}.idl
   ```

3. Update the generated files and `__init__.py` as needed.

## API Reference

### DDSNode

Main class for managing DDS communication (topics, services, actions).

#### `__init__(name, domain_id)`

Create a new DDS node.

**Parameters:**
- `name` (str): Node name (default: "dds_node")
- `domain_id` (int, optional): DDS domain ID (default: from ROS_DOMAIN_ID env var)

#### `dds_create_subscription(topic_name, topic_type, callback, qos)`

Create a subscription to a topic.

**Parameters:**
- `topic_name` (str): Name of the topic
- `topic_type` (Type): Message type class
- `callback` (Callable): Function to call when message arrives
- `qos` (Qos, optional): Quality of Service settings

**Returns:** DataReader instance

#### `dds_create_publisher(topic_name, topic_type, qos)`

Create a publisher for a topic.

**Parameters:**
- `topic_name` (str): Name of the topic
- `topic_type` (Type): Message type class
- `qos` (Qos, optional): Quality of Service settings

**Returns:** DataWriter instance

#### `dds_create_service(service_name, request_type, response_type, callback, qos)`

Create a service server.

**Parameters:**
- `service_name` (str): Name of the service
- `request_type` (Type): Request message type
- `response_type` (Type): Response message type
- `callback` (Callable): Function(request) -> response
- `qos` (Qos, optional): Quality of Service settings

**Returns:** Service dictionary

#### `dds_create_client(service_name, request_type, response_type, qos)`

Create a service client.

**Parameters:**
- `service_name` (str): Name of the service
- `request_type` (Type): Request message type
- `response_type` (Type): Response message type
- `qos` (Qos, optional): Quality of Service settings

**Returns:** Client dictionary

#### `dds_call_service(client, request, timeout)`

Call a service synchronously.

**Parameters:**
- `client`: Client returned by dds_create_client
- `request`: Request message
- `timeout` (float): Timeout in seconds (default: 5.0)

**Returns:** Response message or None

#### `dds_spin()`

Keep the node running (blocks until Ctrl+C).

#### `dds_spin_once(timeout)`

Process callbacks once.

**Parameters:**
- `timeout` (float): Maximum time to wait in seconds

#### `dds_destroy_node()`

Cleanup all resources.

## Contributing

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details.

Before contributing, please review:
- **[CONTRIBUTING.md](CONTRIBUTING.md)** - Contribution guidelines

## Documentation

See [docs/](docs/) for complete documentation:
- **[Topics Guide](docs/guides/topics.md)** - Publishing and subscribing
- **[Services Guide](docs/guides/services.md)** - Service communication
- **[Networking Guide](docs/guides/networking.md)** - Network configuration
- **[Architecture](docs/architecture.md)** - System design

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
