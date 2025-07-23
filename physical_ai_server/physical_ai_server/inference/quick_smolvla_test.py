#!/usr/bin/env python3

import asyncio
import numpy as np
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from inference import LeRobotInference


async def quick_test():
    """Quick SmolVLA test"""
    print("Quick SmolVLA Test")
    print("=" * 30)
    
    # SmolVLA path
    policy_path = "/root/.cache/huggingface/hub/models--ROBOTIS--TurnPackageSmolVLA16k/snapshots/c6a1e396f3676546bc0ad02f79cbfcab5f4e6021"
    
    # Initialize manager
    manager = LeRobotInference(device='cuda')
    
    try:
        # Load policy
        print("Loading SmolVLA...")
        is_valid, msg = manager.validate_policy(policy_path)
        if is_valid:
            manager.load_policy()
            print("✅ SmolVLA loaded!")
        else:
            print(f"❌ Failed: {msg}")
            return
        
        # Quick test with CORRECT camera names and image sizes
        images = {
            'cam_top': np.random.randint(0, 255, (240, 424, 3), dtype=np.uint8),
            'cam_wrist': np.random.randint(0, 255, (240, 424, 3), dtype=np.uint8)
        }
        state = [0.0] * 7  # SmolVLA expects 7-dimensional state
        instruction = "turn the package around"
        
        print(f"Image cameras: {list(images.keys())}")
        print(f"Image shape: {images['cam_top'].shape}")
        print(f"State dimension: {len(state)}")
        
        print("\nTesting inference...")
        action = await manager.predict_async(images, state, instruction)
        print(f"✅ Action received: {len(action)} dimensions")
        print(f"Sample: {action[:5]} ...")
        
        manager.clear_policy()
        print("✅ Test completed!")
        
    except Exception as e:
        print(f"❌ Error: {e}")


if __name__ == "__main__":
    asyncio.run(quick_test())