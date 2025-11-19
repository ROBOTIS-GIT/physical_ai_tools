# Service Communication Guide

Complete guide for creating service servers and clients using robotis_dds_python.

## Quick Start

### Service Server

```python
from robotis_dds_python.tools.dds_node import DDSNode
from robotis_dds_python.idl.physical_ai_interfaces.srv import (
    SetRobotType_Request,
    SetRobotType_Response
)

# Create node
node = DDSNode(name='server', domain_id=30)

# Service callback
def handle_request(request):
    print(f'Request: {request.robot_type}')
    
    valid_types = ['ai_worker', 'omx', 'omy']
    if request.robot_type in valid_types:
        return SetRobotType_Response(
            success=True,
            message=f'Set to {request.robot_type}'
        )
    else:
        return SetRobotType_Response(
            success=False,
            message='Invalid robot type'
        )

# Create service
node.dds_create_service(
    '/set_robot_type',
    SetRobotType_Request,
    SetRobotType_Response,
    handle_request
)

# Keep running
node.dds_spin()
```

### Service Client

```python
from robotis_dds_python.tools.dds_node import DDSNode
from robotis_dds_python.idl.physical_ai_interfaces.srv import (
    SetRobotType_Request,
    SetRobotType_Response
)

# Create node
node = DDSNode(name='client', domain_id=30)

# Create client
client = node.dds_create_client(
    '/set_robot_type',
    SetRobotType_Request,
    SetRobotType_Response
)

# Call service
request = SetRobotType_Request(robot_type='ai_worker')
response = client.call(request, timeout=5.0)

if response:
    print(f'Success: {response.success}')
    print(f'Message: {response.message}')

node.dds_destroy_node()
```

## Complete Examples

See: `examples/service/server.py` and `examples/service/client.py`

**Run Server:**
```bash
python examples/service/server.py
```

**Run Client:**
```bash
python examples/service/client.py ai_worker
```

## Communication with ROS 2

### DDS Server ↔ ROS 2 Client

**DDS Side (Server):**
```bash
python examples/service/server.py
```

**ROS 2 Side (Client):**
```bash
export ROS_DOMAIN_ID=30
ros2 service call /set_robot_type \
    physical_ai_interfaces/srv/SetRobotType \
    "{robot_type: 'ai_worker'}"
```

### ROS 2 Server ↔ DDS Client

**ROS 2 Side (Server):**
```bash
# Create and run your ROS 2 service server
export ROS_DOMAIN_ID=30
ros2 run my_package service_server
```

**DDS Side (Client):**
```bash
python examples/service/client.py ai_worker
```

## Available Service Types

### physical_ai_interfaces
- `SetRobotType` - Set robot type (ai_worker, omx, omy)

### Custom Services
See architecture.md for creating custom service types.

## How Services Work

Services in DDS use a Request-Response pattern over topics:

1. **Request Topic**: `/service_name/Request`
2. **Response Topic**: `/service_name/Response`
3. **GUID Matching**: Each request includes a client GUID for response routing

```
Client                          Server
  |                               |
  |--[Request + GUID]------------>|
  |                               | (Process)
  |<--[Response + GUID]-----------|
  |                               |
```

## Troubleshooting

### Service not found

1. **Check Domain ID**: Must match between client and server
2. **Check Service Name**: Ensure exact match including '/'
3. **Wait for discovery**: Add `time.sleep(1.0)` after creating client

### Timeout errors

1. **Increase timeout**: `client.call(request, timeout=10.0)`
2. **Check server running**: Verify server process is active
3. **Check network**: See `networking.md`

### Type mismatch

Ensure the Request and Response types match exactly:
```python
# Server
node.dds_create_service(
    '/my_service',
    MyRequest_Type,   # Must match
    MyResponse_Type,  # Must match
    callback
)

# Client
client = node.dds_create_client(
    '/my_service',
    MyRequest_Type,   # Must match
    MyResponse_Type   # Must match
)
```

## Best Practices

1. **Use meaningful service names**: `/set_robot_type` not `/srv1`
2. **Validate requests**: Check input in server callback
3. **Handle timeouts**: Always check if response is None
4. **Return appropriate messages**: Include helpful error messages
5. **Match Domain IDs**: Server and client must use same domain
