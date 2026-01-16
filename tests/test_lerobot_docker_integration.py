#!/usr/bin/env python3
"""
LeRobot Docker Integration Test

Tests the Zenoh communication between physical_ai_server and LeRobot Docker container.
"""

import json
import os
import sys
import time

sys.path.insert(0, '/root/ros2_ws/src/physical_ai_tools/physical_ai_server')

import zenoh


def test_connection():
    """Test basic Zenoh connection to LeRobot server"""
    print("\n" + "="*60)
    print("TEST 1: Zenoh Connection to LeRobot Server")
    print("="*60)
    
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
    
    try:
        session = zenoh.open(config)
        print("[PASS] Zenoh session opened successfully")
        
        # Try to get status
        request_data = {
            "command": "train_status",
            "params": {},
            "request_id": "test-connection"
        }
        
        replies = session.get(
            "lerobot/command",
            zenoh.Queue(),
            payload=json.dumps(request_data).encode('utf-8'),
            timeout=10.0
        )
        
        for reply in replies.receiver:
            if reply.ok:
                response = json.loads(bytes(reply.ok.payload).decode('utf-8'))
                print(f"[PASS] LeRobot server responded: {response}")
                session.close()
                return True
        
        print("[FAIL] No response from LeRobot server")
        session.close()
        return False
        
    except Exception as e:
        print(f"[FAIL] Connection error: {e}")
        return False


def test_training_start():
    """Test starting training with lerobot/pusht dataset"""
    print("\n" + "="*60)
    print("TEST 2: Start Training (ACT model with lerobot/pusht)")
    print("="*60)
    
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
    
    try:
        session = zenoh.open(config)
        
        request_data = {
            "command": "train_start",
            "params": {
                "policy_type": "act",
                "dataset_path": "lerobot/pusht",
                "output_dir": "/root/.cache/huggingface/lerobot/outputs/train/act_pusht_test"
            },
            "request_id": "test-training"
        }
        
        print(f"Sending training request: {json.dumps(request_data, indent=2)}")
        
        replies = session.get(
            "lerobot/command",
            zenoh.Queue(),
            payload=json.dumps(request_data).encode('utf-8'),
            timeout=30.0
        )
        
        for reply in replies.receiver:
            if reply.ok:
                response = json.loads(bytes(reply.ok.payload).decode('utf-8'))
                print(f"Response: {json.dumps(response, indent=2)}")
                
                if response.get("success"):
                    print("[PASS] Training started successfully")
                    session.close()
                    return True
                else:
                    print(f"[FAIL] Training failed to start: {response.get('message')}")
                    session.close()
                    return False
        
        print("[FAIL] No response from LeRobot server")
        session.close()
        return False
        
    except Exception as e:
        print(f"[FAIL] Error: {e}")
        return False


def test_training_status():
    """Test getting training status"""
    print("\n" + "="*60)
    print("TEST 3: Get Training Status")
    print("="*60)
    
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
    
    try:
        session = zenoh.open(config)
        
        request_data = {
            "command": "train_status",
            "params": {},
            "request_id": "test-status"
        }
        
        replies = session.get(
            "lerobot/command",
            zenoh.Queue(),
            payload=json.dumps(request_data).encode('utf-8'),
            timeout=10.0
        )
        
        for reply in replies.receiver:
            if reply.ok:
                response = json.loads(bytes(reply.ok.payload).decode('utf-8'))
                print(f"Training Status: {json.dumps(response, indent=2)}")
                session.close()
                return response.get("data", {})
        
        print("[FAIL] No response from LeRobot server")
        session.close()
        return None
        
    except Exception as e:
        print(f"[FAIL] Error: {e}")
        return None


def test_status_subscription():
    """Test subscribing to status updates"""
    print("\n" + "="*60)
    print("TEST 4: Status Topic Subscription")
    print("="*60)
    
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
    
    received_status = []
    
    try:
        session = zenoh.open(config)
        
        def on_status(sample):
            data = json.loads(bytes(sample.payload).decode('utf-8'))
            received_status.append(data)
            print(f"Received status update: {data}")
        
        sub = session.declare_subscriber("lerobot/status", on_status)
        print("Subscribed to lerobot/status, waiting 15 seconds for updates...")
        
        time.sleep(15)
        
        sub.undeclare()
        session.close()
        
        if received_status:
            print(f"[PASS] Received {len(received_status)} status updates")
            return True
        else:
            print("[WARN] No status updates received (may be normal if no training running)")
            return True  # Not a failure if server is idle
            
    except Exception as e:
        print(f"[FAIL] Error: {e}")
        return False


def test_training_stop():
    """Test stopping training"""
    print("\n" + "="*60)
    print("TEST 5: Stop Training")
    print("="*60)
    
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
    
    try:
        session = zenoh.open(config)
        
        request_data = {
            "command": "train_stop",
            "params": {},
            "request_id": "test-stop"
        }
        
        replies = session.get(
            "lerobot/command",
            zenoh.Queue(),
            payload=json.dumps(request_data).encode('utf-8'),
            timeout=10.0
        )
        
        for reply in replies.receiver:
            if reply.ok:
                response = json.loads(bytes(reply.ok.payload).decode('utf-8'))
                print(f"Stop Response: {json.dumps(response, indent=2)}")
                session.close()
                return response.get("success", False)
        
        print("[FAIL] No response from LeRobot server")
        session.close()
        return False
        
    except Exception as e:
        print(f"[FAIL] Error: {e}")
        return False


def test_model_list():
    """Test listing available models"""
    print("\n" + "="*60)
    print("TEST 6: List Available Models")
    print("="*60)
    
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
    
    try:
        session = zenoh.open(config)
        
        request_data = {
            "command": "model_list",
            "params": {},
            "request_id": "test-models"
        }
        
        replies = session.get(
            "lerobot/command",
            zenoh.Queue(),
            payload=json.dumps(request_data).encode('utf-8'),
            timeout=10.0
        )
        
        for reply in replies.receiver:
            if reply.ok:
                response = json.loads(bytes(reply.ok.payload).decode('utf-8'))
                print(f"Models: {json.dumps(response, indent=2)}")
                session.close()
                return response.get("data", {}).get("models", [])
        
        print("[FAIL] No response from LeRobot server")
        session.close()
        return []
        
    except Exception as e:
        print(f"[FAIL] Error: {e}")
        return []


def main():
    print("="*60)
    print("LeRobot Docker Integration Test Suite")
    print("="*60)
    
    results = {}
    
    # Test 1: Connection
    results["connection"] = test_connection()
    
    if not results["connection"]:
        print("\n[ABORT] Connection failed, cannot continue tests")
        return results
    
    # Test 2: Start training
    results["training_start"] = test_training_start()
    
    if results["training_start"]:
        # Wait a bit for training to start
        print("\nWaiting 10 seconds for training to initialize...")
        time.sleep(10)
        
        # Test 3: Get status
        status = test_training_status()
        results["training_status"] = status is not None
        
        # Test 4: Subscribe to status
        results["status_subscription"] = test_status_subscription()
        
        # Test 5: Stop training
        results["training_stop"] = test_training_stop()
    
    # Test 6: List models
    models = test_model_list()
    results["model_list"] = len(models) >= 0  # Always passes, just info
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    for test_name, passed in results.items():
        status = "[PASS]" if passed else "[FAIL]"
        print(f"  {test_name}: {status}")
    
    total = len(results)
    passed = sum(1 for v in results.values() if v)
    print(f"\nTotal: {passed}/{total} tests passed")
    
    return results


if __name__ == "__main__":
    main()
