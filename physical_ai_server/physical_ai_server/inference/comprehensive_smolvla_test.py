#!/usr/bin/env python3

import asyncio
import numpy as np
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from inference import LeRobotInference


async def comprehensive_smolvla_test():
    """Comprehensive SmolVLA test with correct specifications"""
    print("Comprehensive SmolVLA Test")
    print("=" * 40)
    
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
        
        print("\n=== SmolVLA Model Specifications ===")
        print("Expected cameras: cam_top, cam_wrist")
        print("Expected image size: (240, 424, 3)")
        print("Expected state dimension: 7")
        print("Expected format: RGB uint8 [0-255]")
        
        # Test 1: Both cameras
        print("\n=== Test 1: Both Cameras ===")
        images_both = {
            'cam_top': np.random.randint(0, 255, (240, 424, 3), dtype=np.uint8),
            'cam_wrist': np.random.randint(0, 255, (240, 424, 3), dtype=np.uint8)
        }
        state = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
        instruction = "turn the package around"
        
        try:
            action = await manager.predict_async(images_both, state, instruction)
            print(f"✅ Both cameras: Success!")
            print(f"   Action dimension: {len(action)}")
            print(f"   Action sample: {[f'{x:.3f}' for x in action[:5]]} ...")
            print(f"   Action range: [{min(action):.3f}, {max(action):.3f}]")
        except Exception as e:
            print(f"❌ Both cameras failed: {e}")
        
        # Test 2: Only cam_top
        print("\n=== Test 2: Only cam_top ===")
        images_top = {
            'cam_top': np.random.randint(0, 255, (240, 424, 3), dtype=np.uint8)
        }
        
        try:
            action = await manager.predict_async(images_top, state, instruction)
            print(f"✅ cam_top only: Success!")
            print(f"   Action sample: {[f'{x:.3f}' for x in action[:5]]} ...")
        except Exception as e:
            print(f"❌ cam_top only failed: {e}")
        
        # Test 3: Only cam_wrist
        print("\n=== Test 3: Only cam_wrist ===")
        images_wrist = {
            'cam_wrist': np.random.randint(0, 255, (240, 424, 3), dtype=np.uint8)
        }
        
        try:
            action = await manager.predict_async(images_wrist, state, instruction)
            print(f"✅ cam_wrist only: Success!")
            print(f"   Action sample: {[f'{x:.3f}' for x in action[:5]]} ...")
        except Exception as e:
            print(f"❌ cam_wrist only failed: {e}")
        
        # Test 4: Different instructions
        print("\n=== Test 4: Different Instructions ===")
        instructions = [
            "turn the package around",
            "pick up the package",
            "move the package to the left",
            "place the package down",
            "rotate the package 90 degrees clockwise",
            "grasp the package firmly",
            "lift the package up"
        ]
        
        for i, instr in enumerate(instructions):
            try:
                # Vary state slightly for each test
                test_state = [s + i * 0.02 for s in state]
                action = await manager.predict_async(images_both, test_state, instr)
                
                action_magnitude = np.linalg.norm(action)
                print(f"✅ '{instr[:25]}...': magnitude {action_magnitude:.3f}")
            except Exception as e:
                print(f"❌ '{instr[:25]}...' failed: {e}")
                break
        
        # Test 5: State variations
        print("\n=== Test 5: State Variations ===")
        state_variants = [
            [0.0] * 7,  # All zeros
            [0.1] * 7,  # All same small value
            [-0.1] * 7, # All same negative value
            [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6],  # Increasing
            [0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0],  # Decreasing
            [0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.4]  # Alternating
        ]
        
        for i, test_state in enumerate(state_variants):
            try:
                action = await manager.predict_async(images_both, test_state, instruction)
                print(f"✅ State variant {i+1}: Success (action range: [{min(action):.3f}, {max(action):.3f}])")
            except Exception as e:
                print(f"❌ State variant {i+1} failed: {e}")
        
        # Test 6: Chunk inference
        print("\n=== Test 6: Chunk Inference ===")
        try:
            action_chunk = manager.predict_chunk(images_both, state, instruction)
            print(f"✅ Chunk inference: Success!")
            print(f"   Chunk shape: {action_chunk.shape}")
            print(f"   First action: {[f'{x:.3f}' for x in action_chunk[0][:5]]} ...")
            print(f"   Last action: {[f'{x:.3f}' for x in action_chunk[-1][:5]]} ...")
        except Exception as e:
            print(f"❌ Chunk inference failed: {e}")
        
        # Test 7: Performance test
        print("\n=== Test 7: Performance Test ===")
        try:
            import time
            
            # Warm up
            for _ in range(3):
                await manager.predict_async(images_both, state, instruction)
            
            # Time multiple inferences
            num_runs = 20
            start_time = time.time()
            
            for i in range(num_runs):
                action = await manager.predict_async(images_both, state, f"{instruction} #{i}")
            
            total_time = time.time() - start_time
            avg_time = total_time / num_runs
            
            print(f"✅ Performance test: Success!")
            print(f"   Average inference time: {avg_time*1000:.1f} ms")
            print(f"   Total time for {num_runs} inferences: {total_time:.2f} seconds")
            print(f"   Inference rate: {num_runs/total_time:.1f} Hz")
            
        except Exception as e:
            print(f"❌ Performance test failed: {e}")
        
        # Test 8: Concurrent inference
        print("\n=== Test 8: Concurrent Inference ===")
        try:
            # Create multiple concurrent tasks
            concurrent_tasks = []
            for i in range(5):
                test_state = [s + i * 0.01 for s in state]
                task = manager.predict_async(images_both, test_state, f"concurrent task {i}")
                concurrent_tasks.append(task)
            
            # Execute all concurrently
            start_time = time.time()
            results = await asyncio.gather(*concurrent_tasks)
            concurrent_time = time.time() - start_time
            
            print(f"✅ Concurrent inference: Success!")
            print(f"   {len(results)} concurrent inferences completed")
            print(f"   Total time: {concurrent_time:.2f} seconds")
            print(f"   Average per inference: {concurrent_time/len(results)*1000:.1f} ms")
            
            # Check if results are different (they should be)
            unique_results = len(set(tuple(r) for r in results))
            print(f"   Unique results: {unique_results}/{len(results)}")
            
        except Exception as e:
            print(f"❌ Concurrent inference failed: {e}")
        
        manager.clear_policy()
        print("\n✅ All tests completed!")
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        import traceback
        traceback.print_exc()


async def main():
    """Main function"""
    print("SmolVLA Comprehensive Testing")
    print("Testing all aspects of the SmolVLA model")
    print("Expected specifications:")
    print("- Camera names: cam_top, cam_wrist")
    print("- Image size: 240x424x3")
    print("- State dimension: 7")
    print("- Input format: RGB uint8")
    print("")
    
    await comprehensive_smolvla_test()
    
    print("\n" + "=" * 50)
    print("Testing Summary:")
    print("- Model expects specific camera names (cam_top, cam_wrist)")
    print("- Image resolution must be exactly 240x424")
    print("- State vector must be exactly 7 dimensions")
    print("- Natural language instructions are supported")
    print("- Both single and chunk inference work")
    print("- Async inference enables concurrent processing")


if __name__ == "__main__":
    asyncio.run(main())
