#!/usr/bin/env python3
"""
Test script for rosbridge connection and replay service.

Usage:
    # Inside docker container:
    python3 test_rosbridge_connection.py

    # Or with specific host:
    python3 test_rosbridge_connection.py --host localhost --port 9090
"""

import argparse
import asyncio
import json
import socket
import subprocess
import sys
import time

# Check if websockets is available
try:
    import websockets
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False


def print_status(step: str, status: str, message: str = ""):
    """Print formatted status message."""
    icons = {"PASS": "\033[92m[PASS]\033[0m", "FAIL": "\033[91m[FAIL]\033[0m", "INFO": "\033[94m[INFO]\033[0m"}
    icon = icons.get(status, "[????]")
    print(f"{icon} {step}: {message}")


def test_port_open(host: str, port: int) -> bool:
    """Test if port is open."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except Exception as e:
        return False


def test_ros2_node_running() -> bool:
    """Test if physical_ai_server node is running."""
    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            text=True,
            timeout=10
        )
        return "physical_ai_server" in result.stdout
    except Exception:
        return False


def test_ros2_service_exists(service_name: str) -> bool:
    """Test if a ROS2 service exists."""
    try:
        result = subprocess.run(
            ["ros2", "service", "list"],
            capture_output=True,
            text=True,
            timeout=10
        )
        return service_name in result.stdout
    except Exception:
        return False


async def test_websocket_connection(host: str, port: int) -> bool:
    """Test WebSocket connection to rosbridge."""
    if not HAS_WEBSOCKETS:
        print_status("WebSocket", "INFO", "websockets package not installed, skipping")
        return True

    uri = f"ws://{host}:{port}"
    try:
        async with websockets.connect(uri, close_timeout=5) as ws:
            # Send a simple rosbridge message
            msg = {
                "op": "call_service",
                "service": "/rosapi/get_time",
                "type": "rosapi/GetTime"
            }
            await ws.send(json.dumps(msg))

            # Wait for response with timeout
            response = await asyncio.wait_for(ws.recv(), timeout=5)
            data = json.loads(response)
            return "values" in data or "result" in data
    except asyncio.TimeoutError:
        return False
    except Exception as e:
        print_status("WebSocket", "INFO", f"Error: {e}")
        return False


async def test_service_call(host: str, port: int, service: str, service_type: str, request: dict) -> dict:
    """Test a service call via rosbridge."""
    if not HAS_WEBSOCKETS:
        return {"success": False, "error": "websockets not installed"}

    uri = f"ws://{host}:{port}"
    try:
        async with websockets.connect(uri, close_timeout=10, max_size=100_000_000) as ws:
            msg = {
                "op": "call_service",
                "service": service,
                "type": service_type,
                "args": request,
                "id": f"test_{int(time.time())}"
            }

            start_time = time.time()
            await ws.send(json.dumps(msg))

            # Collect all fragments
            fragments = {}
            complete_response = None

            while True:
                try:
                    response = await asyncio.wait_for(ws.recv(), timeout=60)
                    data = json.loads(response)

                    # Check if it's a fragment
                    if "total" in data and "num" in data:
                        fragment_id = data.get("id", "default")
                        if fragment_id not in fragments:
                            fragments[fragment_id] = {"total": data["total"], "parts": {}}
                        fragments[fragment_id]["parts"][data["num"]] = data.get("data", "")

                        # Check if all fragments received
                        if len(fragments[fragment_id]["parts"]) == fragments[fragment_id]["total"]:
                            # Reassemble
                            full_data = ""
                            for i in range(fragments[fragment_id]["total"]):
                                full_data += fragments[fragment_id]["parts"][i]
                            complete_response = json.loads(full_data)
                            break
                    else:
                        complete_response = data
                        break

                except asyncio.TimeoutError:
                    return {"success": False, "error": "timeout"}

            elapsed = time.time() - start_time
            return {
                "success": True,
                "response": complete_response,
                "elapsed_seconds": elapsed
            }

    except Exception as e:
        return {"success": False, "error": str(e)}


def run_tests(host: str, port: int, bag_path: str = None):
    """Run all tests."""
    print("\n" + "="*60)
    print("Physical AI Server Connection Test")
    print("="*60 + "\n")

    all_passed = True

    # Test 1: Port open
    print("[Test 1] Checking if rosbridge port is open...")
    if test_port_open(host, port):
        print_status("Port Check", "PASS", f"Port {port} is open on {host}")
    else:
        print_status("Port Check", "FAIL", f"Port {port} is NOT open on {host}")
        print("\n  -> Make sure rosbridge is running:")
        print("     ros2 launch rosbridge_server rosbridge_websocket_launch.xml")
        all_passed = False
        return all_passed

    # Test 2: ROS2 node running (if running inside container)
    print("\n[Test 2] Checking if physical_ai_server node is running...")
    if test_ros2_node_running():
        print_status("Node Check", "PASS", "physical_ai_server node is running")
    else:
        print_status("Node Check", "INFO", "Could not verify node (may be in different ROS domain)")

    # Test 3: Service exists
    print("\n[Test 3] Checking if replay service exists...")
    if test_ros2_service_exists("/replay/get_data"):
        print_status("Service Check", "PASS", "/replay/get_data service exists")
    else:
        print_status("Service Check", "INFO", "Could not verify service via ros2 cli")

    # Test 4: WebSocket connection
    print("\n[Test 4] Testing WebSocket connection...")
    if HAS_WEBSOCKETS:
        ws_result = asyncio.get_event_loop().run_until_complete(
            test_websocket_connection(host, port)
        )
        if ws_result:
            print_status("WebSocket", "PASS", "Successfully connected to rosbridge")
        else:
            print_status("WebSocket", "FAIL", "Could not connect to rosbridge")
            all_passed = False
    else:
        print_status("WebSocket", "INFO", "Install websockets: pip install websockets")

    # Test 5: Simple service call (browse_file)
    print("\n[Test 5] Testing simple service call (browse_file)...")
    if HAS_WEBSOCKETS:
        result = asyncio.get_event_loop().run_until_complete(
            test_service_call(
                host, port,
                "/browse_file",
                "physical_ai_interfaces/srv/BrowseFile",
                {"action": "get_path", "current_path": "/workspace"}
            )
        )
        if result["success"]:
            print_status("Simple Service", "PASS", f"Response in {result['elapsed_seconds']:.2f}s")
        else:
            print_status("Simple Service", "FAIL", result.get("error", "Unknown error"))
            all_passed = False

    # Test 6: Replay service call (if bag_path provided)
    if bag_path:
        print(f"\n[Test 6] Testing replay service with bag: {bag_path}...")
        if HAS_WEBSOCKETS:
            result = asyncio.get_event_loop().run_until_complete(
                test_service_call(
                    host, port,
                    "/replay/get_data",
                    "physical_ai_interfaces/srv/GetReplayData",
                    {"bag_path": bag_path}
                )
            )
            if result["success"]:
                response = result.get("response", {})
                values = response.get("values", {})
                print_status("Replay Service", "PASS", f"Response in {result['elapsed_seconds']:.2f}s")
                if values:
                    print(f"         - Videos: {len(values.get('video_files', []))}")
                    print(f"         - Joint timestamps: {len(values.get('joint_timestamps', []))}")
                    print(f"         - Action timestamps: {len(values.get('action_timestamps', []))}")
            else:
                print_status("Replay Service", "FAIL", result.get("error", "Unknown error"))
                all_passed = False

    # Summary
    print("\n" + "="*60)
    if all_passed:
        print("\033[92mAll tests passed!\033[0m")
    else:
        print("\033[91mSome tests failed. Check the output above.\033[0m")
    print("="*60 + "\n")

    return all_passed


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test rosbridge connection")
    parser.add_argument("--host", default="localhost", help="Rosbridge host")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port")
    parser.add_argument("--bag", help="Path to rosbag for replay test")

    args = parser.parse_args()

    success = run_tests(args.host, args.port, args.bag)
    sys.exit(0 if success else 1)
