#!/usr/bin/env python3
"""
Test Real-Time Inference Pipeline from LeRobot Docker Container

This test verifies:
1. Model loading from checkpoint
2. Real-time inference loop
3. Action publishing via Zenoh
"""

import json
import sys
import time
import threading
from datetime import datetime

# Add zenoh SDK path
zenoh_sdk_path = "/home/dongyun/main_ws/zenoh_ros2_sdk"
sys.path.insert(0, zenoh_sdk_path)

import zenoh


def send_zenoh_command(command: str, params: dict, request_id: str = "test") -> dict:
    """Send a command via Zenoh and wait for response"""
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
    
    session = zenoh.open(config)
    
    request_data = {
        "command": command,
        "params": params,
        "request_id": request_id
    }
    
    response_data = {"success": False, "message": "No response"}
    response_received = threading.Event()
    
    def on_reply(reply):
        nonlocal response_data
        try:
            if reply.ok:
                response_data = json.loads(bytes(reply.ok.payload).decode('utf-8'))
        except Exception as e:
            response_data = {"success": False, "message": str(e)}
        response_received.set()
    
    session.get(
        "lerobot/command",
        on_reply,
        payload=json.dumps(request_data).encode('utf-8'),
        timeout=60.0  # Longer timeout for model loading
    )
    
    response_received.wait(timeout=65.0)
    session.close()
    
    return response_data


class ActionCollector:
    """Collects actions published by inference"""
    
    def __init__(self):
        self.actions = []
        self.session = None
        self.subscriber = None
        
    def connect(self):
        config = zenoh.Config()
        config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
        self.session = zenoh.open(config)
        return True
    
    def subscribe(self):
        def on_action(sample):
            data = json.loads(bytes(sample.payload).decode('utf-8'))
            self.actions.append({
                "received_at": datetime.now().isoformat(),
                "data": data
            })
            if len(self.actions) <= 5 or len(self.actions) % 30 == 0:
                action = data.get("action", {})
                print(f"[ACTION #{data.get('seq', 0)}] joints={str(action.get('joint_positions', []))[:30]}...")
        
        self.subscriber = self.session.declare_subscriber("lerobot/action", on_action)
        print("Subscribed to lerobot/action")
    
    def close(self):
        if self.subscriber:
            self.subscriber.undeclare()
        if self.session:
            self.session.close()


def get_latest_checkpoint():
    """Find the latest trained checkpoint"""
    # Use a known checkpoint from our training test
    # This path is inside the lerobot container
    checkpoint_path = "/root/.cache/huggingface/lerobot/outputs/train/act_pusht_test_20260116_151454/checkpoints/last/pretrained_model"
    return checkpoint_path


def main():
    print("="*60)
    print("Real-Time Inference Pipeline Test")
    print("="*60)
    print(f"Start Time: {datetime.now().isoformat()}")
    
    # Find a trained model checkpoint
    print("\n1. Finding trained model checkpoint...")
    checkpoint_path = get_latest_checkpoint()
    
    if checkpoint_path is None:
        print("[FAIL] No trained model checkpoint found")
        print("       Please run training first!")
        return False
    
    print(f"[PASS] Found checkpoint: {checkpoint_path}")
    
    # Create action collector
    collector = ActionCollector()
    
    try:
        # Connect to Zenoh
        print("\n2. Connecting to Zenoh...")
        if not collector.connect():
            print("[FAIL] Failed to connect to Zenoh")
            return False
        print("[PASS] Connected to Zenoh")
        
        # Subscribe to action topic
        print("\n3. Subscribing to action topic...")
        collector.subscribe()
        
        # Start inference
        print("\n4. Starting real-time inference...")
        params = {
            "model_path": checkpoint_path,
            "inference_freq": 10  # 10 Hz for testing (lower frequency)
        }
        
        print(f"Request params: {json.dumps(params, indent=2)}")
        response = send_zenoh_command("infer_start", params, "test-inference")
        print(f"Response: {json.dumps(response, indent=2)}")
        
        if not response.get("success"):
            print(f"[FAIL] Failed to start inference: {response.get('message')}")
            return False
        
        print("[PASS] Inference started")
        
        # Collect actions for 10 seconds
        print("\n5. Collecting actions for 10 seconds...")
        time.sleep(10)
        
        # Stop inference
        print("\n6. Stopping inference...")
        stop_response = send_zenoh_command("infer_stop", {}, "test-stop")
        print(f"Stop Response: {stop_response}")
        
        # Analyze results
        print("\n" + "="*60)
        print("TEST RESULTS")
        print("="*60)
        
        print(f"\nActions received: {len(collector.actions)}")
        
        if collector.actions:
            first_action = collector.actions[0]["data"]
            last_action = collector.actions[-1]["data"]
            
            print(f"\nFirst action (seq={first_action.get('seq')}):")
            print(f"  joints: {first_action.get('action', {}).get('joint_positions', [])}")
            
            print(f"\nLast action (seq={last_action.get('seq')}):")
            print(f"  joints: {last_action.get('action', {}).get('joint_positions', [])}")
            
            # Calculate actual frequency
            if len(collector.actions) >= 2:
                first_time = datetime.fromisoformat(collector.actions[0]["received_at"])
                last_time = datetime.fromisoformat(collector.actions[-1]["received_at"])
                duration = (last_time - first_time).total_seconds()
                actual_freq = (len(collector.actions) - 1) / duration if duration > 0 else 0
                print(f"\nActual frequency: {actual_freq:.2f} Hz")
            
            # Check sequence numbers
            sequences = [a["data"].get("seq", 0) for a in collector.actions]
            if sequences == sorted(sequences):
                print("\n[PASS] Action sequence numbers are in order")
            else:
                print("\n[WARN] Action sequence numbers are not in order")
            
            print("\n[SUCCESS] Inference pipeline test completed!")
            return True
        else:
            print("\n[FAIL] No actions received")
            return False
            
    except Exception as e:
        print(f"\n[ERROR] Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        collector.close()


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
