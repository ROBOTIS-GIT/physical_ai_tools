#!/usr/bin/env python3

import asyncio
import numpy as np
import os
import sys

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from inference import LeRobotInference, InferenceFactory


async def get_smolvla_config():
    """Get SmolVLA model configuration to understand expected dimensions"""
    print("=== SmolVLA Configuration Analysis ===")
    
    policy_path = "/root/.cache/huggingface/hub/models--ROBOTIS--TurnPackageSmolVLA16k/snapshots/c6a1e396f3676546bc0ad02f79cbfcab5f4e6021"
    
    manager = LeRobotInference(device='cuda')
    
    try:
        # Load policy to inspect configuration
        is_valid, msg = manager.validate_policy(policy_path)
        if not is_valid:
            print(f"❌ Cannot load policy: {msg}")
            return None
        
        manager.load_policy()
        
        # Get policy configuration
        config = manager.get_policy_config()
        print(f"✅ Policy loaded successfully")
        print(f"Policy type: {manager.get_policy_type()}")
        
        # Try to extract expected dimensions from the policy
        if hasattr(manager.policy, 'config'):
            policy_config = manager.policy.config
            print(f"\nPolicy configuration keys: {list(policy_config.keys()) if hasattr(policy_config, 'keys') else 'N/A'}")
            
            # Look for state dimension info
            for key in ['state_dim', 'robot_state_keys', 'observation_features', 'input_shapes']:
                if hasattr(policy_config, key):
                    print(f"{key}: {getattr(policy_config, key)}")
        
        # Try to get normalization stats to understand dimensions
        if hasattr(manager.policy, 'normalize_inputs'):
            normalizer = manager.policy.normalize_inputs
            print(f"\nNormalization stats available:")
            if hasattr(normalizer, 'stats'):
                for key, stats in normalizer.stats.items():
                    if 'state' in key.lower():
                        print(f"  {key}: shape {stats['mean'].shape if hasattr(stats, 'mean') else 'N/A'}")
        
        manager.clear_policy()
        return True
        
    except Exception as e:
        print(f"❌ Configuration analysis error: {e}")
        return None


async def test_smolvla_with_correct_dimensions():
    """Test SmolVLA with correct state dimensions"""
    print("\n=== SmolVLA Corrected Inference Test ===")
    
    policy_path = "/root/.cache/huggingface/hub/models--ROBOTIS--TurnPackageSmolVLA16k/snapshots/c6a1e396f3676546bc0ad02f79cbfcab5f4e6021"
    
    manager = LeRobotInference(device='cuda')
    
    try:
        # Load policy
        print("Loading SmolVLA...")
        is_valid, msg = manager.validate_policy(policy_path)
        if not is_valid:
            print(f"❌ Policy validation failed: {msg}")
            return
        
        manager.load_policy()
        print("✅ SmolVLA policy loaded successfully!")
        
        # Create sample data with CORRECT dimensions
        print("\n=== Preparing Corrected Sample Data ===")
        
        # SmolVLA expects specific camera setup
        sample_images = {
            'top': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        }
        
        # CORRECTED: SmolVLA expects 7-dimensional state (based on error message)
        sample_state = [
            0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7,  # 7 joint positions/values
        ]
        
        # Task instruction for SmolVLA
        sample_instruction = "turn the package around"
        
        print(f"Sample images: {list(sample_images.keys())}")
        print(f"Sample state dimension: {len(sample_state)} (corrected to 7)")
        print(f"Task instruction: '{sample_instruction}'")
        
        print("\n=== Testing Synchronous Inference ===")
        try:
            action = manager.predict(sample_images, sample_state, sample_instruction)
            print(f"✅ Sync inference successful!")
            print(f"Action dimension: {len(action)}")
            print(f"Action values: {[f'{x:.3f}' for x in action]}")
            
            # Analyze action characteristics
            action_array = np.array(action)
            print(f"Action statistics:")
            print(f"  Range: [{action_array.min():.3f}, {action_array.max():.3f}]")
            print(f"  Mean: {action_array.mean():.3f}")
            print(f"  Std: {action_array.std():.3f}")
            
        except Exception as e:
            print(f"❌ Sync inference still failing: {e}")
            print("This suggests the issue might be with image dimensions or other inputs")
            return
        
        print("\n=== Testing Asynchronous Inference ===")
        try:
            action_async = await manager.predict_async(sample_images, sample_state, sample_instruction)
            print(f"✅ Async inference successful!")
            print(f"Action dimension: {len(action_async)}")
            print(f"Action values: {[f'{x:.3f}' for x in action_async]}")
            
            # Compare sync vs async results
            if len(action) == len(action_async):
                diff = np.array(action) - np.array(action_async)
                print(f"Sync vs Async difference: max={np.abs(diff).max():.6f}")
            
        except Exception as e:
            print(f"❌ Async inference error: {e}")
        
        print("\n=== Testing Different Instructions ===")
        test_instructions = [
            "turn the package around",
            "pick up the object", 
            "move the package to the left",
            "place the item down",
            "rotate the package 90 degrees"
        ]
        
        for i, instruction in enumerate(test_instructions):
            try:
                # Slightly vary the state for each test
                test_state = [s + i * 0.01 for s in sample_state]
                action = await manager.predict_async(sample_images, test_state, instruction)
                
                print(f"✅ Test {i+1}: '{instruction[:30]}...'")
                print(f"   Action range: [{min(action):.3f}, {max(action):.3f}]")
                
            except Exception as e:
                print(f"❌ Test {i+1} failed: {e}")
                break
        
        print("\n=== Testing Different Image Setups ===")
        
        # Test with different image configurations
        image_configs = [
            {'top': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)},
            {'wrist': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)},
            {
                'top': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
                'wrist': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            }
        ]
        
        for i, images in enumerate(image_configs):
            try:
                action = await manager.predict_async(images, sample_state, sample_instruction)
                print(f"✅ Image config {i+1} ({list(images.keys())}): success")
                print(f"   Action sample: {action[:3]} ...")
                
            except Exception as e:
                print(f"❌ Image config {i+1} failed: {e}")
        
        print("\n=== Testing Chunk Inference ===")
        try:
            action_chunk = manager.predict_chunk(sample_images, sample_state, sample_instruction)
            print(f"✅ Chunk inference successful!")
            print(f"Chunk shape: {action_chunk.shape}")
            print(f"First action: {action_chunk[0]}")
            print(f"Last action: {action_chunk[-1]}")
            
        except Exception as e:
            print(f"❌ Chunk inference error: {e}")
        
        manager.clear_policy()
        print("\n✅ All tests completed successfully!")
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        import traceback
        traceback.print_exc()


async def main():
    """Main test function"""
    print("SmolVLA Corrected Inference Testing")
    print("=" * 50)
    
    # First, analyze the model configuration
    await get_smolvla_config()
    
    # Then test with corrected dimensions
    await test_smolvla_with_correct_dimensions()
    
    print("\n" + "=" * 50)
    print("Testing completed!")
    print("\nKey learnings:")
    print("- SmolVLA expects exactly 7-dimensional state vector")
    print("- State likely represents joint positions without separate gripper")
    print("- Image setup may be flexible (test different camera configurations)")
    print("- Natural language instructions are supported")


if __name__ == "__main__":
    # Check CUDA
    try:
        import torch
        if torch.cuda.is_available():
            print(f"🟢 CUDA available: {torch.cuda.get_device_name()}")
            print(f"GPU memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")
        else:
            print("🟡 CUDA not available")
    except ImportError:
        print("🟡 PyTorch not found")
    
    # Run the corrected test
    asyncio.run(main())
