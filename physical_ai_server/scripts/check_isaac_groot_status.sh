#!/bin/bash
# Quick status check for isaac_groot inference server

echo "=== Isaac GR00T Inference Server Status ==="
echo ""

# Check if container is running
if docker ps --format '{{.Names}}' | grep -q "^isaac_gr00t$"; then
    echo "✅ Container: Running"
    
    # Check if server process is running
    if docker exec isaac_gr00t pgrep -f server_inference_async.py > /dev/null 2>&1; then
        echo "✅ Server Process: Running"
        
        # Get PID
        PID=$(docker exec isaac_gr00t pgrep -f server_inference_async.py)
        echo "   PID: $PID"
        
        # Check port
        if docker exec isaac_gr00t netstat -tlnp 2>/dev/null | grep -q ":5555"; then
            echo "✅ Port 5555: Listening"
        else
            echo "⚠️  Port 5555: Not listening"
        fi
        
        # Show last 10 lines of log
        echo ""
        echo "=== Last 10 log lines ==="
        docker exec isaac_gr00t tail -n 10 /tmp/inference_server.log 2>/dev/null || echo "No logs found"
        
    else
        echo "❌ Server Process: Not running"
    fi
else
    echo "❌ Container: Not running"
fi

echo ""
echo "=== Commands ==="
echo "View logs: docker exec isaac_gr00t tail -f /tmp/inference_server.log"
echo "Stop server: docker exec isaac_gr00t pkill -f server_inference_async.py"
