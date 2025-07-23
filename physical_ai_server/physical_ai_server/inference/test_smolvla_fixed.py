#!/usr/bin/env python3

import asyncio
import numpy as np
import os
import sys

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from inference import LeRobotInference, InferenceFactory


async def test_smolvla_inference():
    """Test SmolVLA inference with actual policy"""
    print("=== SmolVLA Inference Test ===")
    
    # SmolVLA policy path
    policy_path = "/root/.cache/huggingface/hub/models--ROBOTIS--TurnPackageSmolVLA16k/snapshots/c6a1e396f3676546bc0ad02f79cbfcab5f4e6021"
    
    # Check if policy exists
    if not os.path.exists(policy_path):
        print(f"❌ Policy path does not exist: {policy_path}")
        print("Please make sure the SmolVLA model is downloaded")
        return
    
    # Initialize LeRobot inference manager
    device = 'cuda'  # SmolVLA typically requires CUDA
    manager = LeRobotInference(device=device)
    
    try:
        # Validate and load policy
        print(f"Validating SmolVLA policy at: {policy_path}")
        is_valid, message = manager.validate_policy(policy_path)
        
        if not is_valid:
            print(f"❌ Policy validation failed: {message}")
            return
        
        print(f"✅ Policy validation successful: {message}")
        print(f"Policy type: {manager.get_policy_type()}")
        
        print("Loading SmolVLA policy...")
        if not manager.load_policy():
            print("❌ Failed to load policy")
            return
        
        print("✅ SmolVLA policy loaded successfully!")
        
        # Get policy config
        config = manager.get_policy_config()
        print(f"Policy configuration loaded")
        
        # Create sample data for SmolVLA - CORRECTED DIMENSIONS
        print("\n=== Preparing Sample Data ===")
        
        # SmolVLA typically uses these camera names (adjust based on your setup)
        sample_images = {
            'top': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
            # 'wrist': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        }
        
        # CORRECTED: SmolVLA expects 7-dimensional state (not 8)
        sample_state = [
            0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7,  # 7 values (likely joint positions)
        ]
        
        # Task instruction for SmolVLA (supports natural language)
        sample_instruction = "turn the package around"
        
        print(f"Sample images: {list(sample_images.keys())}")
        print(f"Sample state dimension: {len(sample_state)} (corrected to 7)")
        print(f"Task instruction: '{sample_instruction}'")
        
        print("\n=== Synchronous Inference Test ===")
        try:
            action = manager.predict(sample_images, sample_state, sample_instruction)
            print(f"✅ Sync inference successful!")
            print(f"Action dimension: {len(action)}")
            print(f"Action sample: {action[:min(5, len(action))]} ...")
            
            # Check action ranges (typical robot action ranges)
            action_array = np.array(action)
            print(f"Action range: [{action_array.min():.3f}, {action_array.max():.3f}]")
            
        except Exception as e:
            print(f"❌ Sync inference error: {e}")
            import traceback
            traceback.print_exc()
        
        print("\n=== Asynchronous Inference Test ===")
        try:
            action_async = await manager.predict_async(sample_images, sample_state, sample_instruction)
            print(f"✅ Async inference successful!")
            print(f"Action dimension: {len(action_async)}")
            print(f"Action sample: {action_async[:min(5, len(action_async))]} ...")
            
        except Exception as e:
            print(f"❌ Async inference error: {e}")
            import traceback
            traceback.print_exc()
        
        print("\n=== Chunk Inference Test ===")
        try:
            action_chunk = manager.predict_chunk(sample_images, sample_state, sample_instruction)
            print(f"✅ Chunk inference successful!")
            print(f"Action chunk shape: {action_chunk.shape}")
            print(f"First action sample: {action_chunk[0][:min(5, action_chunk.shape[1])]} ...")
            print(f"Number of future actions: {action_chunk.shape[0]}")
            
        except Exception as e:
            print(f"❌ Chunk inference error: {e}")
            import traceback
            traceback.print_exc()
        
        print("\n=== Async Chunk Inference Test ===")
        try:
            action_chunk_async = await manager.predict_chunk_async(sample_images, sample_state, sample_instruction)
            print(f"✅ Async chunk inference successful!")
            print(f"Action chunk shape: {action_chunk_async.shape}")
            print(f"First action sample: {action_chunk_async[0][:min(5, action_chunk_async.shape[1])]} ...")
            
        except Exception as e:
            print(f"❌ Async chunk inference error: {e}")
            import traceback
            traceback.print_exc()
        
        print("\n=== Batch Async Inference Test ===")
        try:
            # Test different instructions
            instructions = [
                "turn the package around",
                "pick up the object",
                "place the item carefully"
            ]
            
            # Create multiple inference tasks with different instructions
            tasks = []
            for i, instruction in enumerate(instructions):
                # Slightly modify state for each task (keep 7 dimensions)
                modified_state = [s + i * 0.01 for s in sample_state]
                task = manager.predict_async(sample_images, modified_state, instruction)
                tasks.append(task)
            
            # Execute all tasks concurrently
            print(f"Running {len(tasks)} concurrent inferences...")
            results = await asyncio.gather(*tasks)
            
            print(f"✅ Batch inference completed: {len(results)} results")
            for i, (result, instruction) in enumerate(zip(results, instructions)):
                print(f"  Task {i+1} ('{instruction[:20]}...'): {result[:3]} ... (action dim: {len(result)})")
                
        except Exception as e:
            print(f"❌ Batch async inference error: {e}")
            import traceback
            traceback.print_exc()
        
        print("\n=== Performance Timing Test ===")
        try:
            import time
            
            # Warm up
            for _ in range(3):
                manager.predict(sample_images, sample_state, sample_instruction)
            
            # Time synchronous inference
            num_runs = 10
            start_time = time.time()
            for _ in range(num_runs):
                action = manager.predict(sample_images, sample_state, sample_instruction)
            sync_time = (time.time() - start_time) / num_runs
            
            # Time asynchronous inference
            start_time = time.time()
            tasks = [manager.predict_async(sample_images, sample_state, sample_instruction) for _ in range(num_runs)]
            await asyncio.gather(*tasks)
            async_time = (time.time() - start_time) / num_runs
            
            print(f"✅ Performance comparison:")
            print(f"  Sync inference:  {sync_time*1000:.1f} ms per inference")
            print(f"  Async inference: {async_time*1000:.1f} ms per inference")
            print(f"  Speedup ratio:   {sync_time/async_time:.2f}x")
            
        except Exception as e:
            print(f"❌ Performance test error: {e}")
        
        # Clear policy
        manager.clear_policy()
        print("\n✅ Policy cleared successfully")
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        import traceback
        traceback.print_exc()


async def test_smolvla_factory():
    """Test SmolVLA through factory pattern"""
    print("\n=== SmolVLA Factory Pattern Test ===")
    
    policy_path = "/root/.cache/huggingface/hub/models--ROBOTIS--TurnPackageSmolVLA16k/snapshots/c6a1e396f3676546bc0ad02f79cbfcab5f4e6021"
    
    try:
        # Test available frameworks
        available = InferenceFactory.get_available_frameworks()
        print(f"Available frameworks: {available}")
        
        # Create SmolVLA manager through factory
        manager = InferenceFactory.create_inference_manager('lerobot', device='cuda')
        print(f"✅ Created manager type: {type(manager).__name__}")
        
        # Load SmolVLA policy
        is_valid, msg = manager.validate_policy(policy_path)
        if is_valid:
            manager.load_policy()
            print(f"✅ SmolVLA loaded through factory pattern")
            
            # Test quick inference with CORRECTED dimensions
            sample_images = {'top': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)}
            sample_state = [0.0] * 7  # CORRECTED: 7 dimensions, not 8
            action = await manager.predict_async(sample_images, sample_state, "test command")
            print(f"✅ Factory inference successful: action dim {len(action)}")
            
            manager.clear_policy()
        else:
            print(f"❌ Policy validation failed: {msg}")
        
    except Exception as e:
        print(f"❌ Factory test error: {e}")


async def test_real_world_scenario():
    """Test realistic manipulation scenario"""
    print("\n=== Real-World Manipulation Scenario ===")
    
    policy_path = "/root/.cache/huggingface/hub/models--ROBOTIS--TurnPackageSmolVLA16k/snapshots/c6a1e396f3676546bc0ad02f79cbfcab5f4e6021"
    
    manager = LeRobotInference(device='cuda')
    
    try:
        # Load policy
        is_valid, msg = manager.validate_policy(policy_path)
        if not is_valid:
            print(f"❌ Cannot load policy: {msg}")
            return
        
        manager.load_policy()
        print("✅ SmolVLA ready for manipulation tasks")
        
        # Simulate a manipulation sequence
        manipulation_tasks = [
            ("approach", "move closer to the package"),
            ("grasp", "grasp the package firmly"),
            ("turn", "turn the package around"),
            ("place", "place the package down gently")
        ]
        
        print("\nSimulating manipulation sequence...")
        
        for step, (phase, instruction) in enumerate(manipulation_tasks):
            print(f"\n--- Step {step+1}: {phase.upper()} ---")
            print(f"Instruction: '{instruction}'")
            
            # Simulate robot state evolution - CORRECTED to 7 dimensions
            current_state = [
                0.1 + 0.05 * step,      # Joint 1
                -0.1 - 0.03 * step,     # Joint 2  
                0.05 + 0.02 * step,     # Joint 3
                -0.2 + 0.1 * step,      # Joint 4
                0.3 - 0.05 * step,      # Joint 5
                0.1 + 0.04 * step,      # Joint 6
                -0.1 + 0.06 * step,     # Joint 7
            ]
            
            # Simulate camera images (in real scenario, these would be actual camera feeds)
            images = {
                'top': np.random.randint(50 + step * 30, 200 + step * 30, (480, 640, 3), dtype=np.uint8),
            }
            
            # Get action from SmolVLA
            action = await manager.predict_async(images, current_state, instruction)
            
            print(f"Current state: {[f'{x:.2f}' for x in current_state]}")
            print(f"Predicted action: {[f'{x:.2f}' for x in action[:7]]} ...")
            
            # Simulate action execution feedback
            action_magnitude = np.linalg.norm(action)
            if action_magnitude > 0.1:
                print(f"🟡 Large motion predicted (magnitude: {action_magnitude:.3f})")
            else:
                print(f"🟢 Smooth motion predicted (magnitude: {action_magnitude:.3f})")
        
        print("\n✅ Manipulation sequence simulation completed!")
        manager.clear_policy()
        
    except Exception as e:
        print(f"❌ Real-world scenario test error: {e}")
        import traceback
        traceback.print_exc()


async def main():
    """Main test function for SmolVLA"""
    print("SmolVLA Inference Testing (Corrected)")
    print("=" * 50)
    print("Policy: ROBOTIS TurnPackageSmolVLA16k")
    print("Device: CUDA (GPU required for optimal performance)")
    print("State dimension: 7 (corrected from previous 8)")
    print("=" * 50)
    
    # Test SmolVLA inference
    await test_smolvla_inference()
    
    # Test factory pattern
    await test_smolvla_factory()
    
    # Test real-world scenario
    await test_real_world_scenario()
    
    print("\n" + "=" * 50)
    print("SmolVLA testing completed!")
    print("\nKey findings:")
    print("- SmolVLA expects 7-dimensional state vector (not 8)")
    print("- Single inference: Tests basic SmolVLA functionality")
    print("- Chunk inference: Gets multiple future actions at once")
    print("- Async inference: Enables concurrent processing")
    print("- Factory pattern: Provides consistent interface")
    print("- Real-world scenario: Simulates manipulation sequences")


if __name__ == "__main__":
    # Check CUDA availability
    try:
        import torch
        if torch.cuda.is_available():
            print(f"🟢 CUDA available: {torch.cuda.get_device_name()}")
        else:
            print("🟡 CUDA not available, using CPU (will be slower)")
    except ImportError:
        print("🟡 PyTorch not found, cannot check CUDA")
    
    # Run the test
    asyncio.run(main())
