"""
ZMQ Communication Integration Example

This example shows how to integrate Docker Manager with ZMQ communication
for Physical AI Tools.
"""

import logging
import time
from pathlib import Path

from physical_ai_server.docker_manager import (
    CommunicationProtocol,
    DockerManager,
    FrameworkType,
    MessageType,
    ZMQClient,
    ZMQClientPool,
    ZMQServer,
)


def setup_logging():
    """Setup logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


def example_zmq_server():
    """
    Example: ZMQ Server (runs inside container)
    
    This shows how to implement a ZMQ server inside a framework container.
    """
    print("\n" + "="*80)
    print("Example: ZMQ Server (Framework Container Side)")
    print("="*80)
    
    # Simulated inference function
    def handle_inference(data):
        """Handle inference request."""
        print(f"  Received inference request: {data.keys()}")
        observation = data.get('observation', {})
        
        # Simulate inference
        time.sleep(0.1)
        
        # Return dummy action
        result = {
            'action': [0.1, 0.2, 0.3, 0.4, 0.5],
            'confidence': 0.95,
        }
        print(f"  Returning action: {result['action']}")
        return result
    
    def handle_train(data):
        """Handle training request."""
        print(f"  Received training request: {data.keys()}")
        config = data.get('config', {})
        
        # Simulate training
        return {
            'status': 'started',
            'job_id': 'train_12345',
        }
    
    # Create and run server
    print("\n[1] Starting ZMQ server on tcp://*:5555")
    print("    (This would run inside the container)")
    
    server = ZMQServer('tcp://*:5555')
    server.register_handler(MessageType.INFERENCE, handle_inference)
    server.register_handler(MessageType.TRAIN, handle_train)
    
    print("\n[2] Server ready to accept requests")
    print("    Press Ctrl+C to stop")
    
    try:
        # Run for 30 seconds or until interrupted
        import threading
        server_thread = threading.Thread(target=server.run)
        server_thread.daemon = True
        server_thread.start()
        
        time.sleep(30)
        
    except KeyboardInterrupt:
        print("\n\n[3] Stopping server...")
    finally:
        server.stop()


def example_zmq_client():
    """
    Example: ZMQ Client (Physical AI Server side)
    
    This shows how Physical AI Server communicates with containers.
    """
    print("\n" + "="*80)
    print("Example: ZMQ Client (Physical AI Server Side)")
    print("="*80)
    
    # Note: This requires a running ZMQ server (from example_zmq_server)
    print("\n[1] Connecting to ZMQ server at tcp://localhost:5555")
    print("    Make sure a ZMQ server is running!")
    
    try:
        with ZMQClient('tcp://localhost:5555', timeout=5000) as client:
            
            # Health check
            print("\n[2] Performing health check...")
            if client.health_check():
                print("    ✓ Server is healthy")
            else:
                print("    ✗ Server is not responding")
                return
            
            # Send inference request
            print("\n[3] Sending inference request...")
            observation = {
                'image': [[1, 2, 3], [4, 5, 6]],  # Dummy image
                'robot_state': [0.1, 0.2, 0.3],
            }
            
            response = client.inference(observation)
            
            if response and response.get('status') == 'success':
                result = response['result']
                print(f"    ✓ Received action: {result.get('action')}")
                print(f"    ✓ Confidence: {result.get('confidence')}")
            else:
                print(f"    ✗ Request failed: {response}")
            
            # Send training request
            print("\n[4] Sending training request...")
            config = {
                'dataset': 'my_dataset',
                'epochs': 100,
            }
            
            response = client.train(config)
            
            if response and response.get('status') == 'success':
                result = response['result']
                print(f"    ✓ Training started: {result.get('job_id')}")
            else:
                print(f"    ✗ Request failed: {response}")
            
    except Exception as e:
        print(f"\n✗ Error: {e}")
        print("  Make sure ZMQ server is running (run example_zmq_server first)")


def example_client_pool():
    """
    Example: ZMQ Client Pool for multiple frameworks
    
    This shows how Physical AI Server manages multiple framework connections.
    """
    print("\n" + "="*80)
    print("Example: ZMQ Client Pool (Multiple Frameworks)")
    print("="*80)
    
    print("\n[1] Creating client pool...")
    pool = ZMQClientPool()
    
    # Add clients for different frameworks
    # Note: These endpoints should point to actual running containers
    frameworks = {
        'lerobot': 'tcp://localhost:5555',
        'groot': 'tcp://localhost:5556',
        'pi0': 'tcp://localhost:5557',
    }
    
    print("\n[2] Adding framework clients...")
    for name, endpoint in frameworks.items():
        try:
            pool.add_client(name, endpoint, timeout=5000)
            print(f"    ✓ Added {name}: {endpoint}")
        except Exception as e:
            print(f"    ✗ Failed to add {name}: {e}")
    
    # Health check all frameworks
    print("\n[3] Health check all frameworks...")
    health = pool.health_check_all()
    for name, is_healthy in health.items():
        status = "✓ Healthy" if is_healthy else "✗ Unhealthy"
        print(f"    {name:10s}: {status}")
    
    # Send inference to specific framework
    print("\n[4] Sending inference to LeRobot...")
    observation = {'image': [[1, 2, 3]], 'state': [0.1, 0.2]}
    
    response = pool.inference('lerobot', observation)
    if response and response.get('status') == 'success':
        print(f"    ✓ Received result: {response['result']}")
    else:
        print(f"    ✗ Request failed")
    
    # Cleanup
    print("\n[5] Closing all clients...")
    pool.close_all()


def example_docker_zmq_integration():
    """
    Example: Full integration with Docker Manager
    
    This shows the complete workflow:
    1. Create Docker containers
    2. Start ZMQ servers inside containers
    3. Connect with ZMQ clients
    4. Communicate with frameworks
    """
    print("\n" + "="*80)
    print("Example: Docker + ZMQ Full Integration")
    print("="*80)
    
    manager = DockerManager()
    client_pool = ZMQClientPool()
    
    try:
        # Step 1: Create and start containers
        print("\n[Step 1] Creating framework containers...")
        
        frameworks = [
            (FrameworkType.LEROBOT, 5555),
            (FrameworkType.GROOT_N15, 5556),
        ]
        
        containers = {}
        
        for framework, zmq_port in frameworks:
            try:
                print(f"\n  Creating {framework.value} container...")
                
                # Create container
                container_id = manager.create_container(
                    framework=framework,
                    api_port=zmq_port,  # ZMQ port
                    gpu_ids=[0],
                )
                
                # Start container
                manager.start_container(container_id)
                containers[framework] = container_id
                
                print(f"  ✓ {framework.value} container started: {container_id[:12]}")
                
            except Exception as e:
                print(f"  ✗ Failed: {e}")
        
        # Step 2: Start ZMQ servers inside containers
        print("\n[Step 2] Starting ZMQ servers in containers...")
        
        for framework, container_id in containers.items():
            try:
                print(f"\n  Starting {framework.value} ZMQ server...")
                
                # Start framework process (which includes ZMQ server)
                process_config = {
                    'mode': 'inference',
                    'port': 5555 if framework == FrameworkType.LEROBOT else 5556,
                }
                
                process_info = manager.start_framework_process(
                    container_id=container_id,
                    framework=framework,
                    config=process_config,
                )
                
                print(f"  ✓ ZMQ server started (PID: {process_info['pid']})")
                
            except Exception as e:
                print(f"  ✗ Failed: {e}")
        
        # Wait for servers to be ready
        print("\n[Step 3] Waiting for servers to be ready...")
        time.sleep(3)
        
        # Step 3: Setup ZMQ communication
        print("\n[Step 4] Setting up ZMQ clients...")
        
        for framework, container_id in containers.items():
            try:
                # Get container IP
                container_ip = manager.get_container_ip(container_id)
                zmq_port = 5555 if framework == FrameworkType.LEROBOT else 5556
                endpoint = f"tcp://{container_ip}:{zmq_port}"
                
                # Add to client pool
                client_pool.add_client(framework.value, endpoint)
                
                print(f"  ✓ {framework.value} client: {endpoint}")
                
            except Exception as e:
                print(f"  ✗ Failed: {e}")
        
        # Step 4: Test communication
        print("\n[Step 5] Testing communication...")
        
        # Health check
        health = client_pool.health_check_all()
        print("\n  Health status:")
        for name, is_healthy in health.items():
            status = "✓ Healthy" if is_healthy else "✗ Unhealthy"
            print(f"    {name:10s}: {status}")
        
        # Send inference request
        print("\n  Sending inference request to LeRobot...")
        observation = {
            'image': [[1, 2, 3], [4, 5, 6]],
            'robot_state': [0.1, 0.2, 0.3],
        }
        
        response = client_pool.inference('lerobot', observation)
        
        if response and response.get('status') == 'success':
            result = response['result']
            print(f"    ✓ Received action: {result.get('action')}")
        else:
            print(f"    ✗ Request failed")
        
        # Step 5: Monitor resources
        print("\n[Step 6] Monitoring container resources...")
        
        for framework, container_id in containers.items():
            try:
                resources = manager.monitor_resources(container_id)
                print(f"\n  {framework.value}:")
                print(f"    CPU: {resources.get('cpu_percent', 0):5.1f}%")
                print(f"    Memory: {resources.get('memory_percent', 0):5.1f}%")
            except Exception as e:
                print(f"  ✗ Failed: {e}")
        
        # Cleanup
        print("\n[Step 7] Cleaning up...")
        client_pool.close_all()
        
        for framework, container_id in containers.items():
            manager.stop_container(container_id)
            manager.remove_container(container_id, force=True)
            print(f"  ✓ Removed {framework.value} container")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        client_pool.close_all()
        manager.cleanup()


def main():
    """Run examples."""
    setup_logging()
    
    examples = [
        ("ZMQ Server (Container Side)", example_zmq_server),
        ("ZMQ Client (Server Side)", example_zmq_client),
        ("Client Pool (Multiple Frameworks)", example_client_pool),
        ("Full Docker + ZMQ Integration", example_docker_zmq_integration),
    ]
    
    print("\n" + "="*80)
    print("ZMQ Communication Examples for Physical AI Tools")
    print("="*80)
    print("\nAvailable examples:")
    for i, (name, _) in enumerate(examples, 1):
        print(f"  {i}. {name}")
    print("  0. Exit")
    
    try:
        choice = input(f"\nSelect example to run (0-{len(examples)}): ")
        choice = int(choice)
        
        if choice == 0:
            print("Exiting...")
            return
        elif 1 <= choice <= len(examples):
            name, func = examples[choice - 1]
            print(f"\nRunning example: {name}")
            func()
        else:
            print("Invalid choice")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
