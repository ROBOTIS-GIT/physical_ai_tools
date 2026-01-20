#!/usr/bin/env python3
"""
Test Checkpoint Management API for LeRobot Docker Container

This test verifies that checkpoint listing, info retrieval, and deletion
work correctly via the Zenoh API.

Test Cases:
1. list_checkpoints - Get all available checkpoints
2. checkpoint_info - Get detailed info about a specific checkpoint
3. checkpoint_delete - Delete a checkpoint (with safety checks)
"""

import json
import sys
import time
from datetime import datetime
from pathlib import Path

# Add zenoh SDK path if needed
zenoh_sdk_path = "/home/dongyun/main_ws/zenoh_ros2_sdk"
sys.path.insert(0, zenoh_sdk_path)

import zenoh


def send_zenoh_command(command: str, params: dict, request_id: str = "test", timeout: float = 30.0) -> dict:
    """Send a command via Zenoh and wait for response"""
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
    
    session = zenoh.open(config)
    
    request_data = {
        "command": command,
        "params": params,
        "request_id": request_id
    }
    
    try:
        replies = session.get(
            "lerobot/command",
            zenoh.Queue(),
            payload=json.dumps(request_data).encode('utf-8'),
            timeout=timeout
        )
        
        for reply in replies.receiver:
            if reply.ok:
                return json.loads(bytes(reply.ok.payload).decode('utf-8'))
        
        return {"success": False, "message": "No response from LeRobot server"}
        
    finally:
        session.close()


def test_list_checkpoints():
    """Test listing all available checkpoints"""
    print("\n" + "=" * 60)
    print("TEST: checkpoint_list")
    print("=" * 60)
    
    response = send_zenoh_command("checkpoint_list", {})
    
    print(f"\nResponse: success={response.get('success')}")
    print(f"Message: {response.get('message')}")
    
    if response.get('success'):
        checkpoints = response.get('data', {}).get('checkpoints', [])
        print(f"\nFound {len(checkpoints)} checkpoint(s):")
        
        for i, ckpt in enumerate(checkpoints, 1):
            print(f"\n  [{i}] {ckpt.get('run_name')}/{ckpt.get('checkpoint_name')}")
            print(f"      Policy: {ckpt.get('policy_type')}")
            print(f"      Dataset: {ckpt.get('dataset')}")
            print(f"      Step: {ckpt.get('step')}")
            print(f"      Size: {ckpt.get('size_mb')} MB")
            print(f"      Created: {ckpt.get('created_at')}")
            print(f"      Path: {ckpt.get('path')}")
            if ckpt.get('is_latest'):
                print(f"      [LATEST]")
        
        return checkpoints
    else:
        print(f"Error: {response.get('message')}")
        return []


def test_checkpoint_info(checkpoint_path: str):
    """Test getting detailed checkpoint info"""
    print("\n" + "=" * 60)
    print("TEST: checkpoint_info")
    print("=" * 60)
    print(f"Path: {checkpoint_path}")
    
    response = send_zenoh_command(
        "checkpoint_info", 
        {"checkpoint_path": checkpoint_path}
    )
    
    print(f"\nResponse: success={response.get('success')}")
    print(f"Message: {response.get('message')}")
    
    if response.get('success'):
        data = response.get('data', {})
        print(f"\nCheckpoint Details:")
        print(f"  Run: {data.get('run_name')}")
        print(f"  Checkpoint: {data.get('checkpoint_name')}")
        print(f"  Policy: {data.get('policy_type')}")
        print(f"  Dataset: {data.get('dataset')}")
        print(f"  Size: {data.get('size_mb')} MB")
        
        # Show model files
        files = data.get('files', [])
        if files:
            print(f"\n  Model Files ({len(files)}):")
            for f in files:
                print(f"    - {f.get('name')}: {f.get('size_mb')} MB")
        
        # Show config excerpt if available
        config = data.get('config', {})
        if config:
            print(f"\n  Config excerpt:")
            print(f"    type: {config.get('type')}")
            print(f"    device: {config.get('device')}")
        
        return data
    else:
        print(f"Error: {response.get('message')}")
        return None


def test_checkpoint_delete_dry_run(checkpoint_path: str):
    """Test checkpoint delete - DRY RUN ONLY (shows what would be deleted)"""
    print("\n" + "=" * 60)
    print("TEST: checkpoint_delete (DRY RUN - NOT ACTUALLY DELETING)")
    print("=" * 60)
    print(f"Path: {checkpoint_path}")
    
    # NOTE: This is a DRY RUN - we only verify the path is valid
    # Actual deletion is commented out for safety
    
    # First verify the checkpoint exists
    info_response = send_zenoh_command(
        "checkpoint_info",
        {"checkpoint_path": checkpoint_path}
    )
    
    if info_response.get('success'):
        data = info_response.get('data', {})
        print(f"\nWould delete checkpoint:")
        print(f"  Run: {data.get('run_name')}")
        print(f"  Checkpoint: {data.get('checkpoint_name')}")
        print(f"  Size: {data.get('size_mb')} MB")
        print(f"\n[DRY RUN] Not actually deleting. To delete, uncomment the delete call below.")
        
        # Uncomment to actually delete:
        # delete_response = send_zenoh_command(
        #     "checkpoint_delete",
        #     {"checkpoint_path": checkpoint_path}
        # )
        # print(f"Delete response: {delete_response}")
        
        return True
    else:
        print(f"Checkpoint not found: {info_response.get('message')}")
        return False


def run_all_tests():
    """Run all checkpoint management tests"""
    print("\n" + "#" * 60)
    print("# CHECKPOINT MANAGEMENT TEST SUITE")
    print("# Started at:", datetime.now().isoformat())
    print("#" * 60)
    
    # Test 1: List checkpoints
    checkpoints = test_list_checkpoints()
    
    if not checkpoints:
        print("\n[WARNING] No checkpoints found. Run training first to create checkpoints.")
        print("Skipping checkpoint_info and checkpoint_delete tests.")
        return
    
    # Test 2: Get info for first checkpoint
    first_checkpoint = checkpoints[0]
    checkpoint_path = first_checkpoint.get('path')
    
    if checkpoint_path:
        test_checkpoint_info(checkpoint_path)
    
    # Test 3: Dry run delete (doesn't actually delete)
    if checkpoint_path:
        test_checkpoint_delete_dry_run(checkpoint_path)
    
    print("\n" + "#" * 60)
    print("# TEST SUITE COMPLETE")
    print("#" * 60)


def interactive_mode():
    """Run tests interactively"""
    print("\nCheckpoint Management Interactive Test")
    print("=" * 40)
    print("Commands:")
    print("  list     - List all checkpoints")
    print("  info     - Get checkpoint info (prompts for path)")
    print("  delete   - Delete checkpoint (prompts for path)")
    print("  exit     - Exit")
    print()
    
    while True:
        cmd = input(">>> ").strip().lower()
        
        if cmd == "exit":
            break
        elif cmd == "list":
            test_list_checkpoints()
        elif cmd == "info":
            path = input("Checkpoint path: ").strip()
            if path:
                test_checkpoint_info(path)
        elif cmd == "delete":
            path = input("Checkpoint path to delete: ").strip()
            confirm = input("Are you sure? (yes/no): ").strip().lower()
            if path and confirm == "yes":
                response = send_zenoh_command(
                    "checkpoint_delete",
                    {"checkpoint_path": path}
                )
                print(f"Response: {json.dumps(response, indent=2)}")
            else:
                print("Cancelled.")
        else:
            print("Unknown command. Try: list, info, delete, exit")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Test Checkpoint Management API")
    parser.add_argument(
        "--interactive", "-i",
        action="store_true",
        help="Run in interactive mode"
    )
    parser.add_argument(
        "--list", "-l",
        action="store_true",
        help="Only list checkpoints"
    )
    
    args = parser.parse_args()
    
    if args.interactive:
        interactive_mode()
    elif args.list:
        test_list_checkpoints()
    else:
        run_all_tests()
