# robotis_dds_python System Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         robotis_dds_python                              │
│              (DDS-based communication without ROS 2)                    │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │
        ┌───────────────────────────┼───────────────────────────┐
        │                           │                           │
        ▼                           ▼                           ▼
┌───────────────┐          ┌───────────────┐          ┌───────────────┐
│   DDSNode     │          │ IDL Messages  │          │  CycloneDDS   │
│  (Core API)   │◄─────────┤ (Type Defs)   │          │ (DDS Runtime) │
└───────────────┘          └───────────────┘          └───────────────┘
        │                           │                           │
        │                           │                           │
        └───────────────────────────┴───────────────────────────┘
                                    │
                    ┌───────────────┼───────────────┐
                    │               │               │
                    ▼               ▼               ▼
            ┌──────────┐    ┌──────────┐    ┌──────────┐
            │  Topics  │    │ Services │    │ Actions  │
            └──────────┘    └──────────┘    └──────────┘
                    │               │               │
                    └───────────────┼───────────────┘
                                    │
                                    ▼
                        ┌─────────────────────┐
                        │   ROS 2 Systems     │
                        │   (Compatible)      │
                        └─────────────────────┘
```

## DDSNode Class Structure

```
┌──────────────────────────────────────────────────────────────────┐
│                           DDSNode                                │
├──────────────────────────────────────────────────────────────────┤
│  Properties:                                                     │
│  • name: str                      - Node name                    │
│  • domain_id: int                 - DDS domain ID                │
│  • domain_participant             - DDS participant              │
│  • subscribers: Dict              - Subscriber management        │
│  • publishers: Dict               - Publisher management         │
│  • services: Dict                 - Service server management    │
│  • clients: Dict                  - Service client management    │
├──────────────────────────────────────────────────────────────────┤
│  Topic Methods:                                                  │
│  • dds_create_subscription()      - Create topic subscription    │
│  • dds_create_publisher()         - Create topic publisher       │
├──────────────────────────────────────────────────────────────────┤
│  Service Methods:                                                │
│  • dds_create_service()           - Create service server        │
│  • dds_create_client()            - Create service client        │
│  • dds_call_service()             - Call service (synchronous)   │
├──────────────────────────────────────────────────────────────────┤
│  Lifecycle Methods:                                              │
│  • dds_spin()                     - Run node (blocking)          │
│  • dds_spin_once()                - Run once                     │
│  • dds_destroy_node()             - Cleanup node                 │
└──────────────────────────────────────────────────────────────────┘
```

## Topic Communication Flow

```
┌─────────────────┐                             ┌─────────────────┐
│  Publisher      │                             │  Subscriber     │
│  (DDSNode)      │                             │  (DDSNode)      │
└────────┬────────┘                             └────────┬────────┘
         │                                               │
         │ 1. dds_create_publisher()                     │
         │    topic_name: "/cmd_vel"                     │
         │    topic_type: Twist_                         │
         │                                               │
         ▼                                               │
    ┌────────────┐                                       │
    │ DataWriter │                                       │
    └─────┬──────┘                                       │
          │                                              │
          │ 2. write(msg)                                │
          │                                              │
          ▼                                              │
    ┌──────────────────┐                                 │
    │  DDS Topic:      │                                 │
    │  "rt/cmd_vel"    │                                 │
    └────────┬─────────┘                                 │
             │                                           │
             │ 3. DDS transmission                       │
             │                                           │
             └──────────────────────────────────────────►│
                                                         │
                                         4. dds_create_subscription()
                                            topic_name: "/cmd_vel"
                                            topic_type: Twist_
                                            callback: my_callback
                                                         │
                                                         ▼
                                                  ┌──────────────┐
                                                  │ DataReader   │
                                                  │ + Listener   │
                                                  └──────┬───────┘
                                                         │
                                                         │ 5. on_data_available()
                                                         │
                                                         ▼
                                                  ┌──────────────┐
                                                  │ my_callback  │
                                                  │    (msg)     │
                                                  └──────────────┘
```

## Service Communication Flow

```
┌─────────────────┐                              ┌─────────────────┐
│  Service Client │                              │ Service Server  │
│  (DDSNode)      │                              │  (DDSNode)      │
└────────┬────────┘                              └────────┬────────┘
         │                                                │
         │ 1. dds_create_client()                         │ 1. dds_create_service()
         │    service: "/set_robot_type"              │    service: "/set_robot_type"
         │    req: SetRobotType_Request               │    req: SetRobotType_Request
         │    res: SetRobotType_Response              │    res: SetRobotType_Response
         │                                                │    callback: handle_request
         ▼                                                ▼
    ┌─────────────────────┐                     ┌─────────────────────┐
    │ Request Writer      │                     │ Request Reader      │
    │ Response Reader     │                     │ Response Writer     │
    └──────┬──────────────┘                     └──────┬──────────────┘
           │                                           │
           │ 2. dds_call_service(req)                  │
           │    req = SetRobotType_Request(            │
           │             robot_type='ai_worker')       │
           │                                           │
           ▼                                           │
    ┌────────────────────────┐                         │
    │ Request Topic:         │                         │
    │ "rq/set_robot_type     │                         │
    │      Request"          │                         │
    └─────────┬──────────────┘                         │
              │                                        │
              │ 3. DDS transmission                    │
              │                                        │
              └───────────────────────────────────────►│
                                                       │
                                       4. CallbackListener.on_data_available()
                                                       │
                                                       ▼
                                              ┌─────────────────┐
                                              │ handle_request  │
                                              │   (request)     │
                                              │                 │
                                              │ response.success│
                                              │     = True      │
                                              └────────┬────────┘
                                                       │
                                                       │ 5. return response
                                                       │
                                                       ▼
                                              ┌─────────────────┐
                                              │ Response Topic: │
                                              │ "rr/set_robot_  │
                                              │    type Reply"  │
                                              └────────┬────────┘
                                                       │
                                                       │ 6. DDS transmission
                                                       │
           ◄───────────────────────────────────────────┘
           │
           │ 7. response_reader.take()
           │
           ▼
    ┌──────────────────┐
    │ Return response  │
    │ to caller        │
    └──────────────────┘
```

## CallbackListener Mechanism

```
┌──────────────────────────────────────────────────────────────┐
│                    CallbackListener                          │
├──────────────────────────────────────────────────────────────┤
│  Extends: cyclonedds.core.Listener                           │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────────────────────────────────────────┐     │
│  │  on_data_available(reader)                          │     │
│  │  ─────────────────────────────────────────────────  │     │
│  │  1. samples = reader.take(N=100)                    │     │
│  │     - Take up to 100 samples at once                │     │
│  │                                                     │     │
│  │  2. for sample in samples:                          │     │
│  │     - Filter InvalidSample (disposal notifications) │     │
│  │     - Call callback(sample)                         │     │
│  │                                                     │     │
│  │  3. Repeat until all samples processed              │     │
│  └─────────────────────────────────────────────────────┘     │
│                                                              │
│  Benefits:                                                   │
│  • Asynchronous callback processing                          │
│  • Batch message processing                                  │
│  • Same usage pattern as ROS 2                               │
└──────────────────────────────────────────────────────────────┘
```

## IDL Message Type Structure

```
robotis_dds_python/
└── idl/
    ├── std_msgs/
    │   └── msg/
    │       ├── String_.py
    │       ├── Int32_.py
    │       └── __init__.py
    │
    ├── geometry_msgs/
    │   └── msg/
    │       ├── Twist_.py
    │       ├── Vector3_.py
    │       └── __init__.py
    │
    ├── trajectory_msgs/
    │   └── msg/
    │       ├── JointTrajectory_.py
    │       ├── JointTrajectoryPoint_.py
    │       └── __init__.py
    │
    └── physical_ai_interfaces/
        └── srv/
            ├── SetRobotType.idl      ← IDL definition
            ├── SetRobotType.py       ← Generated by idlc
            │   - SetRobotType_Request
            │   - SetRobotType_Response
            └── __init__.py

Generation Process:
  1. Write IDL file (SetRobotType.idl)
  2. Run idlc compiler
     $ idlc -l py SetRobotType.idl
  3. Use generated Python module
     from robotis_dds_python.idl.physical_ai_interfaces.srv import (
         SetRobotType_Request, SetRobotType_Response
     )
```

## ROS 2 Topic Naming Convention

```
User Topic Name        →    DDS Topic Name
───────────────────────────────────────────
"/cmd_vel"            →    "rt/cmd_vel"
"/joint_states"       →    "rt/joint_states"
"cmd_vel"             →    "rt/cmd_vel"

ROS 2 adds "rt/" prefix to all topics.
DDSNode._normalize_topic_name() handles this automatically.
```

## ROS 2 Service Implementation: Topic-Based Pattern

### Why Services Use Topics

ROS 2 Services are **not native DDS services** - they are implemented using **2 Topics** (Request + Response) because:

1. **DDS Standard Only Provides Pub-Sub**: The DDS specification only defines Publisher-Subscriber pattern, not Request-Response
2. **Network Transparency**: Topic-based approach works seamlessly across different networks
3. **Scalability**: Multiple clients and servers can interact simultaneously
4. **Tooling Compatibility**: Can be monitored with standard DDS tools (Wireshark, ddsperf, etc.)

### Service Naming Convention

```
Service Name: "/set_robot_type"

Request Topic:  "rq/set_robot_typeRequest"
                ├─ "rq" = Request prefix (from service name)
                └─ "Request" = Request type suffix

Response Topic: "rr/set_robot_typeReply"
                ├─ "rr" = Reply prefix (from service name)
                └─ "Reply" = Response type suffix

This follows the ROS 2 DDS standard convention.
```

### Detailed Service Communication Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│              How ROS 2 Service Works (Topic-Based)                  │
└─────────────────────────────────────────────────────────────────────┘

[Client Node]                                        [Server Node]
     │                                                     │
     │ ① Create Client                                    │ ① Create Service
     │    • Publisher:   "rq/set_robot_typeRequest"       │    • Subscriber: "rq/set_robot_typeRequest"
     │    • Subscriber:  "rr/set_robot_typeReply"         │    • Publisher:  "rr/set_robot_typeReply"
     │                                                     │
     │ ② Call service(request)                            │
     │    request.robot_type = 'ai_worker'                │
     │    request.client_guid = [UUID]  ◄──────────────────┼─── Unique identifier
     │    request.sequence_number = 1                      │
     │                                                     │
     │ ③ PUBLISH to Request Topic                         │
     │    ────────────────────────────────────────────────►│
     │         Topic: "rq/add_two_intsRequest"             │
     │                                                     │
     │                                  ④ SUBSCRIBE receives request
     │                                     (CallbackListener triggers)
     │                                                     │
     │                                  ⑤ Execute service callback
     │                                     def handle_request(req):
     │                                         return Response(sum=req.a+req.b)
     │                                                     │
     │                                  ⑥ Create response
     │                                     response.sum = 30
     │                                     response.client_guid = [UUID]  ← Same as request
     │                                     response.sequence_number = 1
     │                                                      │
     │                                  ⑦ PUBLISH to Response Topic
     │    ◄──────────────────────────────────────────────── │
     │         Topic: "rr/add_two_intsReply"                │
     │                                                      │
     │ ⑧ SUBSCRIBE receives response                       │
     │    • Read all responses from topic                   │
     │    • FILTER by client_guid:                          │
     │       ✓ Match → Return to caller                     │
     │       ✗ Not match → Ignore (other client's response) │
     │                                                      │
     │ ⑨ Return response to application                    │
     │    print(f"Success: {response.success}")            │
     │    print(f"Message: {response.message}")            │
     │                                                      │
```

### GUID-Based Response Matching

Since **multiple clients** can call the same service simultaneously, each request/response includes a **GUID (Globally Unique Identifier)**:

```python
# Internal Request Message Structure
class SetRobotType_Request:
    client_guid: bytes[16]      # Client's unique identifier
    sequence_number: int64      # Request sequence number
    # User-defined fields:
    robot_type: string

# Internal Response Message Structure  
class SetRobotType_Response:
    client_guid: bytes[16]      # Which client sent the request
    sequence_number: int64      # Which request this responds to
    # User-defined fields:
    success: bool
    message: string
```

**How it works:**
1. Client generates unique GUID when created
2. Client publishes request with its GUID
3. Server publishes response with **same GUID**
4. Client filters responses: only processes messages with matching GUID
5. Other clients' responses are ignored

### Multi-Client Scenario Example

```
Client A (GUID: aaa)  ────►  Service Topic  ◄────  Server
                              "rq/set_robot_        │
Client B (GUID: bbb)  ────►  type Request"         │
                                                    │
                                                    ▼
                              Process both requests
                                                    │
                                                    ▼
Client A  ◄──── (GUID: aaa) ─┐                      │
                             ├─ Response Topic      │
Client B  ◄──── (GUID: bbb) ─┘  "rr/set_robot_  ◄───┘
                                 type Reply"

• Client A only processes response with GUID "aaa"
• Client B only processes response with GUID "bbb"
• Both share same Request/Response topics
```

## Execution Flow Examples

### Topic Example
```python
# 1. Create node
node = DDSNode(name="example_node", domain_id=0)

# 2. Create publisher
pub = node.dds_create_publisher("/cmd_vel", Twist_)

# 3. Create subscriber (register callback)
def callback(msg):
    print(f"Received: {msg}")

sub = node.dds_create_subscription("/sensor_data", Data_, callback)

# 4. Publish message
msg = Twist_(linear=Vector3_(x=1.0), angular=Vector3_(z=0.5))
pub.write(msg)

# 5. Run node (activate callbacks)
node.dds_spin()  # Exit with Ctrl+C
```

### Service Example
```python
# Server
from robotis_dds_python.tools.dds_node import DDSNode
from robotis_dds_python.idl.physical_ai_interfaces.srv import (
    SetRobotType_Request,
    SetRobotType_Response
)

server_node = DDSNode(name="server")

def handle_set_robot_type(request):
    print(f"Setting robot type to: {request.robot_type}")
    return SetRobotType_Response(
        success=True,
        message=f"Robot type set to {request.robot_type}"
    )

service = server_node.dds_create_service(
    "/set_robot_type",
    SetRobotType_Request,
    SetRobotType_Response,
    handle_set_robot_type
)
server_node.dds_spin()

# Client (separate process)
client_node = DDSNode(name="client")
client = client_node.dds_create_client(
    "/set_robot_type",
    SetRobotType_Request,
    SetRobotType_Response
)

request = SetRobotType_Request(robot_type='ai_worker')
response = client_node.dds_call_service(client, request, timeout=5.0)
print(f"Success: {response.success}")
print(f"Message: {response.message}")
```

## System Features

### 1. **ROS 2 Independence**
- No ROS 2 installation required
- Works with CycloneDDS only
- Runs in lightweight environments

### 2. **ROS 2 Compatibility**
- Uses ROS 2 message types
- Follows ROS 2 naming conventions
- Transparently communicates with ROS 2 systems

### 3. **Ease of Use**
- ROS 2-like API
- Callback-based asynchronous processing
- Single node manages multiple communications

### 4. **Extensibility**
- Add new message types via IDL compiler
- Configurable QoS settings
- Domain isolation support

## Dependency Structure

```
robotis_dds_python
       │
       ├─ Python 3.8+
       │
       ├─ CycloneDDS 0.10.2
       │      │
       │      ├─ libddsc.so (C library)
       │      └─ idlc (IDL compiler)
       │
       └─ cyclonedds-python 0.10.5
              │
              ├─ cyclonedds.core
              ├─ cyclonedds.domain
              ├─ cyclonedds.sub
              ├─ cyclonedds.pub
              └─ cyclonedds.topic
```

## Network Configuration for Remote Communication

### Cross-Network Communication (Robot ↔ Cloud Server)

CycloneDDS supports communication across different networks through proper configuration. Here's how to set it up:

#### 1. **Using XML Configuration File**

Create a CycloneDDS configuration file to specify network interfaces and peers:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer address="CLOUD_SERVER_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

**Save this as:** `cyclonedds.xml`

**Apply the configuration:**
```bash
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
```

#### 2. **Configuration Parameters Explained**

- **NetworkInterfaceAddress**: Network interface to use
  - `auto` - Automatically select interface
  - Specific IP (e.g., `192.168.1.100`) - Use specific interface
  
- **AllowMulticast**: Enable/disable multicast discovery
  - `false` - Disable multicast (required for WAN communication)
  - `true` - Enable multicast (LAN only)
  
- **Peers**: Specify remote DDS participants
  - Add cloud server IP addresses for direct peer discovery
  - Multiple peers can be added

#### 3. **Example Configurations**

**Robot Configuration (Edge Device):**
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="cloud.example.com:7400"/>
        <Peer address="52.12.34.56:7400"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

**Cloud Server Configuration:**
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>0.0.0.0</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Ports>
        <Base>7400</Base>
      </Ports>
    </Discovery>
  </Domain>
</CycloneDDS>
```

#### 4. **Using Environment Variables**

For simple configurations, use environment variables directly:

```bash
# Robot side
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><Peers><Peer Address="cloud-server-ip:7400"/></Peers></Discovery></Domain></CycloneDDS>'

# Cloud server side
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>'
```

#### 5. **Python Code Example**

```python
import os
from robotis_dds_python.tools.dds_node import DDSNode
from robotis_dds_python.idl.std_msgs.msg import String_

# Set configuration before creating node
os.environ['CYCLONEDDS_URI'] = 'file:///path/to/cyclonedds.xml'

# Or set inline
os.environ['CYCLONEDDS_URI'] = '''
<CycloneDDS>
  <Domain>
    <Discovery>
      <Peers>
        <Peer address="cloud-server.com:7400"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
'''

# Create node (will use the configuration)
node = DDSNode(name="robot_node", domain_id=0)

# Now can communicate with cloud server
pub = node.dds_create_publisher("/robot/status", String_)
pub.write(String_(data="Robot online"))

node.dds_spin()
```

#### 6. **Network Requirements**

**Firewall Rules:**
- Open UDP port 7400 (default DDS discovery port)
- Open UDP port 7410-7420 (data transmission ports)
- Or use port range: 7400-7500

**NAT Traversal:**
- For NAT/firewall environments, consider using:
  - Port forwarding on router
  - VPN tunnel between robot and cloud
  - DDS routing service (Eclipse Cyclone DDS Router)

**Cloud Security Group (AWS/GCP/Azure):**
```bash
# Allow inbound UDP traffic for DDS
Inbound Rule:
  - Protocol: UDP
  - Port Range: 7400-7500
  - Source: Robot IP or 0.0.0.0/0 (for dynamic IPs)
```

#### 7. **Testing Connection**

**Test script:**
```python
#!/usr/bin/env python3
import os
import time
from robotis_dds_python.tools.dds_node import DDSNode
from robotis_dds_python.idl.std_msgs.msg import String_

# Set peer configuration
os.environ['CYCLONEDDS_URI'] = '''
<CycloneDDS>
  <Domain>
    <General><AllowMulticast>false</AllowMulticast></General>
    <Discovery>
      <Peers><Peer address="REMOTE_IP:7400"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
'''

node = DDSNode(name="test_node")

# Publisher
pub = node.dds_create_publisher("/test/ping", String_)

# Subscriber
def callback(msg):
    print(f"Received: {msg.data}")

sub = node.dds_create_subscription("/test/pong", String_, callback)

# Send ping messages
for i in range(10):
    pub.write(String_(data=f"Ping {i}"))
    print(f"Sent: Ping {i}")
    time.sleep(1)

node.dds_destroy_node()
```

Run this on both robot and cloud server (swap ping/pong topics) to test connectivity.

#### 8. **Common Network Topologies**

**Scenario 1: Direct Internet Connection**
```
┌──────────┐                    ┌──────────────┐
│  Robot   │◄──── Internet ────►│ Cloud Server │
│ (Public) │    UDP 7400-7500   │  (Public IP) │
└──────────┘                    └──────────────┘

Configuration: Direct peer-to-peer with public IPs
```

**Scenario 2: Robot Behind NAT**
```
┌──────────┐    ┌─────┐                ┌──────────────┐
│  Robot   │◄───┤ NAT │◄── Internet ───┤ Cloud Server │
│ (Private)│    └─────┘   Port Forward │  (Public IP) │
└──────────┘                            └──────────────┘

Configuration: Port forwarding on NAT, cloud server as peer
```

**Scenario 3: VPN Tunnel**
```
┌──────────┐                            ┌──────────────┐
│  Robot   │◄────── VPN Tunnel ────────►│ Cloud Server │
│ 10.0.0.2 │      (Private Network)     │  10.0.0.1    │
└──────────┘                            └──────────────┘

Configuration: Use VPN private IPs as peers
```

**Scenario 4: DDS Router (Recommended for Complex Networks)**
```
┌──────────┐    ┌─────────────┐    ┌──────────────┐
│  Robot   │◄───┤ DDS Router  │◄───┤ Cloud Server │
│  (Edge)  │    │  (Gateway)  │    │   (Cloud)    │
└──────────┘    └─────────────┘    └──────────────┘

Configuration: Central router handles routing between networks
```

#### 9. **Performance Tuning**

For WAN communication, adjust these settings:

```xml
<CycloneDDS>
  <Domain>
    <Internal>
      <MinimumSocketReceiveBufferSize>10MB</MinimumSocketReceiveBufferSize>
    </Internal>
    <Discovery>
      <SPDPInterval>30s</SPDPInterval>
      <LeaseDuration>60s</LeaseDuration>
    </Discovery>
  </Domain>
</CycloneDDS>
```

- **SPDPInterval**: How often to send discovery messages (increase for WAN)
- **LeaseDuration**: Timeout for participant liveness (increase for unreliable networks)
- **SocketReceiveBufferSize**: Increase for high-bandwidth applications

#### 10. **Troubleshooting**

**Check if DDS discovery is working:**
```bash
# Install CycloneDDS tools
sudo apt install cyclonedds-tools  # or build from source

# Monitor DDS traffic
ddsperf sanity

# List discovered participants
ddsls -a
```

**Common issues:**
- **No communication**: Check firewall rules and peer addresses
- **Intermittent connection**: Increase LeaseDuration and SPDPInterval
- **High latency**: Use QoS settings with appropriate reliability
- **Packet loss**: Enable RTPS reliability with retransmission

## Key Component Relationship Diagram

```
┌────────────────────────────────────────────────────────────┐
│                      Application Layer                     │
│                                                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │ Publisher    │  │ Subscriber   │  │ Service      │   │
│  │ Example      │  │ Example      │  │ Example      │   │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘   │
└─────────┼──────────────────┼──────────────────┼───────────┘
          │                  │                  │
          └──────────────────┼──────────────────┘
                             │
┌────────────────────────────┼────────────────────────────────┐
│                      DDSNode API Layer                      │
│                            │                                │
│  ┌─────────────────────────┴─────────────────────────┐    │
│  │  • dds_create_subscription()                      │    │
│  │  • dds_create_publisher()                         │    │
│  │  • dds_create_service()                           │    │
│  │  • dds_create_client()                            │    │
│  │  • dds_spin() / dds_spin_once()                   │    │
│  └───────────────────────┬───────────────────────────┘    │
└──────────────────────────┼──────────────────────────────────┘
                           │
┌──────────────────────────┼──────────────────────────────────┐
│                 CycloneDDS Python Bindings                  │
│                          │                                  │
│  ┌───────────────────────┴───────────────────────┐         │
│  │  • DomainParticipant                          │         │
│  │  • Publisher / Subscriber                     │         │
│  │  • DataWriter / DataReader                    │         │
│  │  • Topic / Listener                           │         │
│  └───────────────────────┬───────────────────────┘         │
└──────────────────────────┼──────────────────────────────────┘
                           │
┌──────────────────────────┼──────────────────────────────────┐
│                    CycloneDDS Core (C)                      │
│                          │                                  │
│  ┌───────────────────────┴───────────────────────┐         │
│  │  • DDS Discovery                              │         │
│  │  • RTPS Protocol                              │         │
│  │  • Network Transport                          │         │
│  │  • QoS Management                             │         │
│  └───────────────────────┬───────────────────────┘         │
└──────────────────────────┼──────────────────────────────────┘
                           │
                           ▼
                    ┌──────────────┐
                    │   Network    │
                    │  (UDP/TCP)   │
                    └──────────────┘
```

---

This architecture diagram illustrates the complete structure of robotis_dds_python. It is a lightweight solution that enables seamless communication with ROS 2 systems via DDS without requiring ROS 2 installation.
