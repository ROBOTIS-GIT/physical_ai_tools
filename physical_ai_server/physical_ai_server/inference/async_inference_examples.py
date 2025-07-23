#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Dongyun Kim

# Example usage of async inference functionality

import asyncio
import numpy as np
from physical_ai_server.inference import InferenceFactory

async def single_async_inference_example():
    # Example: Single async inference
    print("=== Single Async Inference Example ===")
    
    manager = InferenceFactory.create_inference_manager('lerobot')
    
    # Mock data
    images = {
        'camera1': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        'camera2': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    }
    state = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    task_instruction = "pick up the red cube"
    
    # Async single-step inference
    action = await manager.predict_async(images, state, task_instruction)
    print(f"Predicted action: {action}")
    
    # Async chunk-based inference
    action_chunk = await manager.predict_chunk_async(images, state, task_instruction)
    print(f"Predicted action chunk shape: {action_chunk.shape}")

async def batch_async_inference_example():
    # Example: Multiple async inferences running concurrently
    print("\n=== Batch Async Inference Example ===")
    
    manager = InferenceFactory.create_inference_manager('lerobot')
    
    # Create multiple inference tasks
    tasks = []
    for i in range(5):
        images = {
            'camera1': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        }
        state = [0.1 * i, 0.2 * i, 0.3 * i, 0.4 * i, 0.5 * i, 0.6 * i, 0.7 * i]
        
        # Create both single and chunk inference tasks
        single_task = manager.predict_async(images, state, f"task_{i}")
        chunk_task = manager.predict_chunk_async(images, state, f"task_{i}_chunk")
        
        tasks.extend([single_task, chunk_task])
    
    # Execute all tasks concurrently
    print("Starting concurrent inference tasks...")
    results = await asyncio.gather(*tasks)
    
    print(f"Completed {len(results)} inference tasks")
    for i, result in enumerate(results):
        if isinstance(result, list):
            print(f"Task {i//2} single result: {len(result)} actions")
        else:
            print(f"Task {i//2} chunk result: {result.shape}")

async def mixed_framework_async_example():
    # Example: Using different frameworks asynchronously
    print("\n=== Mixed Framework Async Example ===")
    
    # Create managers for different frameworks
    lerobot_manager = InferenceFactory.create_inference_manager('lerobot')
    pi_manager = InferenceFactory.create_inference_manager('physical_intelligence')
    groot_manager = InferenceFactory.create_inference_manager('groot')
    
    # Mock data
    images = {'camera1': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)}
    state = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    
    # Run inference on all frameworks concurrently
    tasks = [
        lerobot_manager.predict_async(images, state, "lerobot task"),
        pi_manager.predict_async(images, state, "pi task"),
        groot_manager.predict_async(images, state, "groot task")
    ]
    
    print("Running inference on multiple frameworks...")
    results = await asyncio.gather(*tasks)
    
    frameworks = ['LeRobot', 'Physical Intelligence', 'GR00T']
    for framework, result in zip(frameworks, results):
        print(f"{framework} result: {len(result)} actions")

async def streaming_inference_example():
    # Example: Streaming-like inference processing
    print("\n=== Streaming Inference Example ===")
    
    manager = InferenceFactory.create_inference_manager('lerobot')
    
    async def process_observation_stream():
        # Simulate continuous observation stream
        for i in range(10):
            # Generate mock observation
            images = {'camera1': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)}
            state = [0.1 * i] * 7
            
            # Process observation
            action = await manager.predict_async(images, state, f"stream_step_{i}")
            print(f"Stream step {i}: action = {action[:3]}...")  # Show first 3 actions
            
            # Simulate time delay between observations
            await asyncio.sleep(0.1)
    
    await process_observation_stream()

async def main():
    # Run all examples
    await single_async_inference_example()
    await batch_async_inference_example()
    await mixed_framework_async_example()
    await streaming_inference_example()

if __name__ == "__main__":
    # Note: This example assumes policies are loaded and available
    # In practice, you would need to load actual policies first
    print("Async Inference Examples")
    print("Note: This example uses placeholder implementations")
    print("In practice, you need to load actual policies first\n")
    
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"Example error (expected with placeholder implementations): {e}")
        print("This is normal - replace with actual policy loading for real usage")
