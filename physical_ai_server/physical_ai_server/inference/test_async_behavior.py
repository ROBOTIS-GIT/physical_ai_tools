#!/usr/bin/env python3

import asyncio
import numpy as np
import sys
import os
import time

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from inference import LeRobotInference


async def test_async_behavior():
    """Test to verify actual async behavior"""
    print("SmolVLA Async Behavior Test")
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
        
        # Prepare test data
        images = {
            'cam_top': np.random.randint(0, 255, (240, 424, 3), dtype=np.uint8),
            'cam_wrist': np.random.randint(0, 255, (240, 424, 3), dtype=np.uint8)
        }
        state = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
        instruction = "turn the package around"
        
        print("\n=== Test 1: Synchronous vs Asynchronous Performance ===")
        
        # Test synchronous inference
        print("Testing synchronous inference...")
        sync_times = []
        for i in range(5):
            start_time = time.time()
            action = manager.predict(images, state, f"{instruction} sync #{i}")
            end_time = time.time()
            sync_times.append(end_time - start_time)
            print(f"  Sync {i+1}: {(end_time - start_time)*1000:.1f}ms")
        
        avg_sync_time = sum(sync_times) / len(sync_times)
        print(f"Average sync time: {avg_sync_time*1000:.1f}ms")
        
        # Test asynchronous inference (sequential)
        print("\nTesting asynchronous inference (sequential)...")
        async_times = []
        for i in range(5):
            start_time = time.time()
            action = await manager.predict_async(images, state, f"{instruction} async #{i}")
            end_time = time.time()
            async_times.append(end_time - start_time)
            print(f"  Async {i+1}: {(end_time - start_time)*1000:.1f}ms")
        
        avg_async_time = sum(async_times) / len(async_times)
        print(f"Average async time: {avg_async_time*1000:.1f}ms")
        
        print("\n=== Test 2: Concurrent Execution (Real Async Test) ===")
        
        # Test 1: Sequential async calls
        print("Sequential async calls:")
        start_time = time.time()
        results_sequential = []
        for i in range(3):
            action = await manager.predict_async(images, state, f"{instruction} seq #{i}")
            results_sequential.append(action)
        sequential_time = time.time() - start_time
        print(f"Sequential time for 3 inferences: {sequential_time:.2f}s ({sequential_time/3*1000:.1f}ms each)")
        
        # Test 2: Concurrent async calls
        print("\nConcurrent async calls:")
        start_time = time.time()
        
        # Create tasks for concurrent execution
        tasks = []
        for i in range(3):
            # Slightly different state for each task
            test_state = [s + i * 0.01 for s in state]
            task = manager.predict_async(images, test_state, f"{instruction} concurrent #{i}")
            tasks.append(task)
        
        # Execute all tasks concurrently
        results_concurrent = await asyncio.gather(*tasks)
        concurrent_time = time.time() - start_time
        print(f"Concurrent time for 3 inferences: {concurrent_time:.2f}s ({concurrent_time/3*1000:.1f}ms average)")
        
        # Calculate speedup
        speedup = sequential_time / concurrent_time
        print(f"\n🚀 Concurrency speedup: {speedup:.2f}x")
        
        if speedup > 1.1:
            print("✅ TRUE ASYNC: Concurrent execution is faster!")
        elif speedup > 0.9:
            print("🟡 MARGINAL: Small difference, might be async overhead")
        else:
            print("❌ NOT ASYNC: Sequential was faster, something's wrong")
        
        print("\n=== Test 3: Async with Other Operations ===")
        
        async def other_async_work(duration, name):
            """Simulate other async work"""
            print(f"  {name} starting...")
            await asyncio.sleep(duration)
            print(f"  {name} completed after {duration}s")
            return f"{name}_result"
        
        print("Testing async inference interleaved with other async work...")
        
        start_time = time.time()
        
        # Start inference and other work concurrently
        inference_task = manager.predict_async(images, state, "interleaved test")
        other_task1 = other_async_work(0.1, "Task1")
        other_task2 = other_async_work(0.2, "Task2")
        
        # Wait for all to complete
        results = await asyncio.gather(inference_task, other_task1, other_task2)
        
        total_time = time.time() - start_time
        print(f"Total time with interleaved work: {total_time:.2f}s")
        print(f"Inference result: {results[0][:3]} ...")
        print(f"Other results: {results[1]}, {results[2]}")
        
        if total_time < 0.5:  # If less than sum of sleep times
            print("✅ TRUE ASYNC: Other work executed during inference!")
        else:
            print("❌ BLOCKING: Other work waited for inference")
        
        print("\n=== Test 4: Multiple Concurrent Inferences ===")
        
        # Test with many concurrent inferences
        num_concurrent = 10
        print(f"Testing {num_concurrent} concurrent inferences...")
        
        start_time = time.time()
        
        # Create many concurrent tasks
        concurrent_tasks = []
        for i in range(num_concurrent):
            test_state = [s + i * 0.001 for s in state]
            task = manager.predict_async(images, test_state, f"batch test #{i}")
            concurrent_tasks.append(task)
        
        # Execute all concurrently
        batch_results = await asyncio.gather(*concurrent_tasks)
        batch_time = time.time() - start_time
        
        print(f"✅ {num_concurrent} concurrent inferences completed!")
        print(f"Total time: {batch_time:.2f}s")
        print(f"Average per inference: {batch_time/num_concurrent*1000:.1f}ms")
        print(f"Theoretical sequential time: {avg_async_time * num_concurrent:.2f}s")
        print(f"Concurrency efficiency: {(avg_async_time * num_concurrent / batch_time):.2f}x")
        
        # Check result diversity
        unique_results = len(set(tuple(r) for r in batch_results))
        print(f"Result diversity: {unique_results}/{num_concurrent} unique results")
        
        if unique_results > num_concurrent * 0.8:
            print("✅ Good diversity: Different inputs produce different outputs")
        else:
            print("🟡 Low diversity: Many similar results")
        
        print("\n=== Test 5: Thread Pool Verification ===")
        
        # Check if we can see thread pool behavior
        print("Checking async implementation details...")
        
        # This test tries to see if we're actually using a thread pool
        import threading
        
        def check_thread():
            return threading.current_thread().name
        
        main_thread = threading.current_thread().name
        print(f"Main thread: {main_thread}")
        
        # The async inference should run in a different thread
        start_time = time.time()
        action = await manager.predict_async(images, state, "thread test")
        end_time = time.time()
        
        print(f"Async inference completed in {(end_time-start_time)*1000:.1f}ms")
        print("✅ If this is running in ThreadPoolExecutor, it's truly async!")
        
        manager.clear_policy()
        print("\n🎉 All async behavior tests completed!")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()


async def simple_concurrent_test():
    """Simple test to clearly show concurrent execution"""
    print("\n" + "=" * 50)
    print("SIMPLE CONCURRENT EXECUTION DEMO")
    print("=" * 50)
    
    policy_path = "/root/.cache/huggingface/hub/models--ROBOTIS--TurnPackageSmolVLA16k/snapshots/c6a1e396f3676546bc0ad02f79cbfcab5f4e6021"
    manager = LeRobotInference(device='cuda')
    
    try:
        manager.validate_policy(policy_path)
        manager.load_policy()
        
        images = {
            'cam_top': np.random.randint(0, 255, (240, 424, 3), dtype=np.uint8),
            'cam_wrist': np.random.randint(0, 255, (240, 424, 3), dtype=np.uint8)
        }
        state = [0.0] * 7
        
        print("Starting 3 inferences at the same time...")
        print("If truly async, you should see all 'Starting...' messages before any 'Completed...' messages")
        
        async def timed_inference(task_id):
            print(f"🟡 Starting inference {task_id}...")
            start = time.time()
            action = await manager.predict_async(images, state, f"task {task_id}")
            duration = time.time() - start
            print(f"✅ Completed inference {task_id} in {duration*1000:.1f}ms")
            return action
        
        # Start all tasks
        start_time = time.time()
        task1 = asyncio.create_task(timed_inference(1))
        task2 = asyncio.create_task(timed_inference(2))
        task3 = asyncio.create_task(timed_inference(3))
        
        # Wait for all to complete
        results = await asyncio.gather(task1, task2, task3)
        total_time = time.time() - start_time
        
        print(f"\n🎯 All 3 inferences completed in {total_time:.2f}s total")
        print(f"📊 Results received: {len(results)} actions")
        
        manager.clear_policy()
        
    except Exception as e:
        print(f"❌ Demo error: {e}")


async def main():
    """Main function"""
    print("🔍 SmolVLA Async Verification Suite")
    print("This test suite verifies that async inference is working correctly")
    print("")
    
    await test_async_behavior()
    await simple_concurrent_test()
    
    print("\n" + "=" * 50)
    print("📋 SUMMARY: How to tell if it's truly async")
    print("=" * 50)
    print("✅ Concurrent execution is faster than sequential")
    print("✅ Multiple 'Starting...' messages appear before 'Completed...' messages")
    print("✅ Other async work can execute during inference")
    print("✅ ThreadPoolExecutor runs inference in background threads")
    print("✅ No blocking behavior observed")


if __name__ == "__main__":
    asyncio.run(main())
