#!/usr/bin/env python3
"""
Test Training Log Publishing from LeRobot Docker Container

This test verifies that training logs (step, loss, gradient) are properly
published via Zenoh from the LeRobot container to physical_ai_server.
"""

import json
import sys
import time
import threading
from datetime import datetime

# Add zenoh SDK path if needed
zenoh_sdk_path = "/home/dongyun/main_ws/zenoh_ros2_sdk"
sys.path.insert(0, zenoh_sdk_path)

import zenoh


class TrainingLogCollector:
    """Collects training logs from Zenoh topics"""
    
    def __init__(self):
        self.status_logs = []
        self.training_logs = []
        self.session = None
        self.subscribers = []
        
    def connect(self):
        config = zenoh.Config()
        config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
        self.session = zenoh.open(config)
        return True
    
    def subscribe(self):
        # Subscribe to status topic
        def on_status(sample):
            data = json.loads(bytes(sample.payload).decode('utf-8'))
            self.status_logs.append({
                "received_at": datetime.now().isoformat(),
                "data": data
            })
            print(f"[STATUS] {data.get('status')}: step={data.get('step', 'N/A')}, loss={data.get('loss', 'N/A')}")
        
        # Subscribe to training log topic
        def on_training_log(sample):
            data = json.loads(bytes(sample.payload).decode('utf-8'))
            self.training_logs.append({
                "received_at": datetime.now().isoformat(),
                "data": data
            })
            metrics = data.get("metrics", {})
            print(f"[TRAIN_LOG] step={metrics.get('step')}, loss={metrics.get('loss'):.4f}, grad={metrics.get('gradient_norm'):.2f}")
        
        sub_status = self.session.declare_subscriber("lerobot/status", on_status)
        sub_training = self.session.declare_subscriber("lerobot/training_log", on_training_log)
        
        self.subscribers = [sub_status, sub_training]
        print("Subscribed to lerobot/status and lerobot/training_log")
    
    def close(self):
        for sub in self.subscribers:
            sub.undeclare()
        if self.session:
            self.session.close()


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
        timeout=30.0
    )
    
    # Wait for response
    response_received.wait(timeout=35.0)
    session.close()
    
    return response_data


def start_training_500_steps():
    """Start training with 500 steps"""
    print("\n" + "="*60)
    print("Starting Training: 500 steps with ACT model on lerobot/pusht")
    print("="*60)
    
    # Use timestamp to ensure unique output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    params = {
        "policy_type": "act",
        "dataset_path": "lerobot/pusht",
        "output_dir": f"/root/.cache/huggingface/lerobot/outputs/train/act_pusht_test_{timestamp}",
        "save_freq": 500  # Save checkpoint every 500 steps
    }
    
    print(f"Request params: {json.dumps(params, indent=2)}")
    
    response = send_zenoh_command("train_start", params, "test-training-500")
    print(f"Response: {json.dumps(response, indent=2)}")
    
    return response.get("success", False)


def stop_training():
    """Stop training"""
    response = send_zenoh_command("train_stop", {}, "test-stop")
    print(f"Stop Response: {response}")
    return response.get("success", False)


def main():
    print("="*60)
    print("Training Log Publishing Test")
    print("="*60)
    print(f"Start Time: {datetime.now().isoformat()}")
    
    # Create log collector
    collector = TrainingLogCollector()
    
    try:
        # Connect to Zenoh
        print("\n1. Connecting to Zenoh...")
        if not collector.connect():
            print("[FAIL] Failed to connect to Zenoh")
            return False
        print("[PASS] Connected to Zenoh")
        
        # Start subscriptions
        print("\n2. Setting up subscriptions...")
        collector.subscribe()
        
        # Start training
        print("\n3. Starting training (500 steps)...")
        if not start_training_500_steps():
            print("[FAIL] Failed to start training")
            return False
        print("[PASS] Training started")
        
        # Wait for training logs (500 steps takes roughly 2-3 minutes with GPU)
        print("\n4. Collecting training logs...")
        print("   (Waiting up to 5 minutes for 500 steps...)")
        
        max_wait = 300  # 5 minutes
        check_interval = 10  # Check every 10 seconds
        waited = 0
        
        while waited < max_wait:
            time.sleep(check_interval)
            waited += check_interval
            
            # Check if training completed or reached 500 steps
            if collector.training_logs:
                last_log = collector.training_logs[-1]
                step = last_log["data"].get("metrics", {}).get("step", 0)
                status = last_log["data"].get("status", "")
                
                print(f"   [{waited}s] Current step: {step}, Status: {status}")
                
                if step >= 500 or status in ["completed", "stopped", "failed"]:
                    print(f"   Training reached target or completed at step {step}")
                    break
        
        # Stop training if still running
        print("\n5. Stopping training...")
        stop_training()
        time.sleep(2)
        
        # Analyze results
        print("\n" + "="*60)
        print("TEST RESULTS")
        print("="*60)
        
        print(f"\nStatus updates received: {len(collector.status_logs)}")
        print(f"Training logs received: {len(collector.training_logs)}")
        
        # Check if we received meaningful training logs
        if collector.training_logs:
            first_log = collector.training_logs[0]["data"]
            last_log = collector.training_logs[-1]["data"]
            
            first_step = first_log.get("metrics", {}).get("step", 0)
            last_step = last_log.get("metrics", {}).get("step", 0)
            first_loss = first_log.get("metrics", {}).get("loss", 0)
            last_loss = last_log.get("metrics", {}).get("loss", 0)
            
            print(f"\nFirst log: step={first_step}, loss={first_loss:.4f}")
            print(f"Last log:  step={last_step}, loss={last_loss:.4f}")
            
            # Verify loss decreased (training is working)
            if last_step > first_step:
                print(f"\n[PASS] Training progressed from step {first_step} to {last_step}")
            else:
                print(f"\n[WARN] Training did not progress much")
            
            if last_loss < first_loss:
                print(f"[PASS] Loss decreased from {first_loss:.4f} to {last_loss:.4f}")
            else:
                print(f"[WARN] Loss did not decrease (may need more steps)")
            
            # Check if status includes step/loss info
            if collector.status_logs:
                sample_status = collector.status_logs[-1]["data"]
                if "step" in sample_status and "loss" in sample_status:
                    print(f"\n[PASS] Status topic includes step and loss metrics")
                else:
                    print(f"\n[WARN] Status topic missing step/loss metrics")
                    print(f"       Status keys: {list(sample_status.keys())}")
            
            print("\n[SUCCESS] Training log publishing test completed!")
            return True
        else:
            print("\n[FAIL] No training logs received")
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
