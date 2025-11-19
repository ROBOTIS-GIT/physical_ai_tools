# Documentation

Complete documentation for robotis_dds_python.

## Quick Start

New to robotis_dds_python? Start here:

1. **[Installation](../README.md#installation)** - Install dependencies and package
2. **[Topics Guide](guides/topics.md)** - Publish and subscribe to messages
3. **[Services Guide](guides/services.md)** - Create service servers and clients
4. **[Networking Guide](guides/networking.md)** - Configure network communication

## Guides

### Core Features
- **[Topics](guides/topics.md)** - Complete guide for topic communication
  - Publisher and subscriber examples
  - ROS 2 integration
  - QoS configuration
  - Troubleshooting

- **[Services](guides/services.md)** - Complete guide for service communication
  - Server and client examples
  - Request-response patterns
  - ROS 2 integration
  - Best practices

- **[Networking](guides/networking.md)** - Network configuration for all scenarios
  - Same computer / LAN / WAN / Internet
  - Firewall configuration
  - NAT traversal and VPN
  - Troubleshooting network issues

### Advanced
- **[Architecture](architecture.md)** - System design and internal workings
  - DDS architecture
  - Message type system
  - Service implementation
  - Creating custom types

## Examples

### Basic Examples
```bash
# Topic Publisher
python examples/topic/publisher.py

# Topic Subscriber
python examples/topic/subscriber.py

# Service Server
python examples/service/server.py

# Service Client
python examples/service/client.py ai_worker
```

### Advanced Examples
See `examples/advanced/` directory:
- `multi_topic.py` - Multiple topics in one node
- `trajectory_publisher.py` - JointTrajectory messages
- `bidirectional_bridge.py` - Full DDS ↔ ROS 2 bridge
- `network_cloud.py` - Cloud server example
- `network_robot.py` - Robot client example

## Quick Reference

### Domain ID
```python
# DDS
node = DDSNode(name='my_node', domain_id=30)

# ROS 2
export ROS_DOMAIN_ID=30
```

### Topic Publishing
```python
pub = node.dds_create_publisher('/topic', String_)
pub.publish(String_(data='Hello'))
```

### Topic Subscribing
```python
def callback(msg):
    print(msg.data)

node.dds_create_subscription('/topic', String_, callback)
node.dds_spin()
```

### Service Server
```python
def handle(request):
    return Response(success=True)

node.dds_create_service('/service', Request, Response, handle)
node.dds_spin()
```

### Service Client
```python
client = node.dds_create_client('/service', Request, Response)
response = client.call(Request(...), timeout=5.0)
```

## Common Issues

| Problem | Solution | Guide |
|---------|----------|-------|
| Topics not visible | Check Domain ID | [Networking](guides/networking.md#troubleshooting) |
| Service timeout | Check network & server running | [Services](guides/services.md#troubleshooting) |
| High latency | Optimize QoS settings | [Topics](guides/topics.md#qos-configuration) |
| Network issues | Configure firewall & peers | [Networking](guides/networking.md#firewall-configuration) |

## API Reference

### DDSNode
Main class for creating DDS nodes.

```python
DDSNode(
    name: str,                    # Node name
    domain_id: int = 0,          # DDS domain ID (must match ROS 2)
    peers: List[str] = None,     # Remote peer addresses
    allow_multicast: bool = True, # Enable multicast discovery
    network_interface: str = 'auto',  # Network interface
    port_base: int = 7400        # DDS port base
)
```

### Methods

**Topic Communication:**
- `dds_create_publisher(topic_name, topic_type, qos)` → DDSPublisher
- `dds_create_subscription(topic_name, topic_type, callback, qos)` → DataReader

**Service Communication:**
- `dds_create_service(service_name, request_type, response_type, callback)` → Service
- `dds_create_client(service_name, request_type, response_type)` → Client

**Node Management:**
- `dds_spin()` - Keep node running
- `dds_destroy_node()` - Cleanup resources

## Contributing

See [CONTRIBUTING.md](../CONTRIBUTING.md) for development guidelines.

## License

Apache 2.0 - See [LICENSE](../LICENSE) for details.
