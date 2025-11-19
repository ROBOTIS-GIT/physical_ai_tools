# Topic Communication Guide

Complete guide for publishing and subscribing to topics using robotis_dds_python.

## Quick Start

### Publisher

```python
from robotis_dds_python.tools.dds_node import DDSNode
from robotis_dds_python.idl.std_msgs.msg import String_

# Create node
node = DDSNode(name='publisher', domain_id=30)

# Create publisher
pub = node.dds_create_publisher('/my_topic', String_)

# Publish message
pub.publish(String_(data='Hello World'))

# Cleanup
node.dds_destroy_node()
```

### Subscriber

```python
from robotis_dds_python.tools.dds_node import DDSNode
from robotis_dds_python.idl.std_msgs.msg import String_

# Create node
node = DDSNode(name='subscriber', domain_id=30)

# Define callback
def callback(msg):
    print(f'Received: {msg.data}')

# Create subscriber
node.dds_create_subscription('/my_topic', String_, callback)

# Keep running
node.dds_spin()
```

## Complete Example

See: `examples/topic/publisher.py` and `examples/topic/subscriber.py`

**Run Publisher:**
```bash
python examples/topic/publisher.py
```

**Run Subscriber:**
```bash
python examples/topic/subscriber.py
```

## Available Message Types

### std_msgs
- `String_` - Simple string messages

### sensor_msgs
- Coming soon

### Custom Messages
See architecture.md for creating custom message types.

## Communication with ROS 2

### From DDS to ROS 2

**DDS Side:**
```python
node = DDSNode(name='dds_pub', domain_id=30)
pub = node.dds_create_publisher('/test_topic', String_)
pub.publish(String_(data='Hello ROS 2'))
```

**ROS 2 Side:**
```bash
export ROS_DOMAIN_ID=30
ros2 topic echo /test_topic std_msgs/msg/String
```

### From ROS 2 to DDS

**ROS 2 Side:**
```bash
export ROS_DOMAIN_ID=30
ros2 topic pub /ros2_test std_msgs/msg/String "data: 'Hello DDS'"
```

**DDS Side:**
```python
node = DDSNode(name='dds_sub', domain_id=30)

def callback(msg):
    print(f'From ROS 2: {msg.data}')

node.dds_create_subscription('/ros2_test', String_, callback)
node.dds_spin()
```

## QoS Configuration

```python
from cyclonedds.core import Policy, Qos
from cyclonedds.util import duration

# Custom QoS
qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.TransientLocal,
    Policy.History.KeepLast(100)
)

pub = node.dds_create_publisher('/topic', String_, qos=qos)
sub = node.dds_create_subscription('/topic', String_, callback, qos=qos)
```

## Troubleshooting

### Messages not received

1. **Check Domain ID**: Must match on both sides
   ```bash
   # ROS 2
   echo $ROS_DOMAIN_ID
   
   # DDS
   # Check domain_id parameter in DDSNode()
   ```

2. **Check Network**: See `networking.md` for network configuration

3. **Check Topic Name**: 
   ```bash
   # ROS 2: List topics
   ros2 topic list
   ```

### High Latency

- Use `Policy.Reliability.BestEffort` for lower latency
- Reduce `History.KeepLast` buffer size
- Check network configuration

## Best Practices

1. **Use appropriate QoS** for your use case
2. **Match Domain ID** between DDS and ROS 2
3. **Clean up resources** with `node.dds_destroy_node()`
4. **Handle errors** in callbacks gracefully
