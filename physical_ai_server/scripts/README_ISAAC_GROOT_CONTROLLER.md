# Isaac GR00T Inference Server Controller

This script allows controlling the inference server (`server_inference_async.py`) running in the `isaac_gr00t` container from within the `physical_ai_server` container.

## Prerequisites

1. **Docker socket must be mounted** in `physical_ai_server` container (already configured in docker-compose.yml)

2. **Install Docker CLI** in the `physical_ai_server` container:
   ```bash
   docker exec -it physical_ai_server bash
   apt-get update && apt-get install -y docker.io
   ```

## Usage

### From physical_ai_server container:

```bash
# Enter the container
docker exec -it physical_ai_server bash

# Navigate to scripts directory
cd /root/ros2_ws/src/physical_ai_tools/physical_ai_server/scripts

# Make script executable
chmod +x control_isaac_groot_server.py

# Check status
python3 control_isaac_groot_server.py status

# Start the server
python3 control_isaac_groot_server.py start

# Start with custom host/port
python3 control_isaac_groot_server.py start --host 0.0.0.0 --port 5556

# Stop the server
python3 control_isaac_groot_server.py stop

# Restart the server
python3 control_isaac_groot_server.py restart

# View logs
python3 control_isaac_groot_server.py logs

# View more log lines
python3 control_isaac_groot_server.py logs --lines 100
```

## Integration with ROS Node

You can integrate this into your ROS node:

```python
import subprocess

def start_inference_server():
    """Start the isaac_gr00t inference server."""
    result = subprocess.run(
        ["python3", "/root/ros2_ws/src/physical_ai_tools/physical_ai_server/scripts/control_isaac_groot_server.py", "start"],
        capture_output=True,
        text=True
    )
    return result.returncode == 0

def stop_inference_server():
    """Stop the isaac_gr00t inference server."""
    result = subprocess.run(
        ["python3", "/root/ros2_ws/src/physical_ai_tools/physical_ai_server/scripts/control_isaac_groot_server.py", "stop"],
        capture_output=True,
        text=True
    )
    return result.returncode == 0

def check_inference_server_status():
    """Check if inference server is running."""
    result = subprocess.run(
        ["python3", "/root/ros2_ws/src/physical_ai_tools/physical_ai_server/scripts/control_isaac_groot_server.py", "status"],
        capture_output=True,
        text=True
    )
    return "✅ Running" in result.stdout
```

## Troubleshooting

### "Docker CLI not found"
Install docker CLI in the container:
```bash
apt-get update && apt-get install -y docker.io
```

### "Permission denied"
Make sure the docker socket is mounted and accessible:
```bash
ls -la /var/run/docker.sock
```

### Server won't start
Check the logs:
```bash
python3 control_isaac_groot_server.py logs
# or directly:
docker exec isaac_gr00t cat /tmp/inference_server.log
```

### Container not found
Make sure isaac_gr00t container is running:
```bash
docker ps | grep isaac_gr00t
```

## Notes

- The server runs in the background using `nohup`
- Logs are saved to `/tmp/inference_server.log` inside the isaac_gr00t container
- The script uses `pgrep` and `pkill` to manage the process
- Default host is `localhost` and port is `5555`
