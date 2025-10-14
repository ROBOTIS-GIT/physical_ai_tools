"""
Example Usage of Docker Manager for Physical AI Tools

This script demonstrates how to use the DockerManager to manage
different Physical AI framework containers.
"""

import logging
from pathlib import Path

from physical_ai_server.docker_manager import (
    CommunicationProtocol,
    ContainerStatus,
    DockerManager,
    FrameworkType,
)


def setup_logging():
    """Setup logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


def example_basic_usage():
    """Example: Basic container lifecycle management."""
    print("\n" + "="*80)
    print("Example 1: Basic Container Lifecycle")
    print("="*80)
    
    # Initialize Docker Manager
    manager = DockerManager(
        network_name="physical_ai_network",
        shared_volume_path=Path.home() / ".cache/physical_ai_tools",
    )
    
    try:
        # 1. Build or pull LeRobot image
        print("\n[1] Pulling LeRobot image...")
        try:
            manager.pull_image(FrameworkType.LEROBOT, tag="latest")
        except Exception as e:
            print(f"Warning: Could not pull image - {e}")
            print("You need to build the image first")
        
        # 2. Create container
        print("\n[2] Creating LeRobot container...")
        container_id = manager.create_container(
            framework=FrameworkType.LEROBOT,
            gpu_ids=[0],  # Use GPU 0
            memory_limit='8g',
            api_port=8000,
        )
        print(f"Created container: {container_id}")
        
        # 3. Start container
        print("\n[3] Starting container...")
        manager.start_container(container_id)
        
        # 4. Check status
        status = manager.get_container_status(container_id)
        print(f"Container status: {status}")
        
        # 5. Execute command
        print("\n[4] Executing test command...")
        output = manager.exec_command(container_id, "echo 'Hello from container'")
        print(f"Output: {output}")
        
        # 6. List all containers
        print("\n[5] Listing all containers...")
        containers = manager.list_containers()
        for container in containers:
            print(f"  - {container['name']}: {container['status']}")
        
        # 7. Monitor resources
        print("\n[6] Monitoring resources...")
        resources = manager.monitor_resources(container_id)
        print(f"  CPU: {resources.get('cpu_percent', 0)}%")
        print(f"  Memory: {resources.get('memory_percent', 0)}%")
        
        # 8. Stop container
        print("\n[7] Stopping container...")
        manager.stop_container(container_id)
        
        # 9. Remove container
        print("\n[8] Removing container...")
        manager.remove_container(container_id, force=True)
        
    finally:
        manager.cleanup()


def example_multi_framework():
    """Example: Running multiple frameworks simultaneously."""
    print("\n" + "="*80)
    print("Example 2: Multiple Framework Containers")
    print("="*80)
    
    manager = DockerManager()
    
    try:
        # Create containers for different frameworks
        frameworks = [
            (FrameworkType.LEROBOT, 8000),
            (FrameworkType.GROOT_N15, 8001),
            (FrameworkType.PI0, 8002),
        ]
        
        container_ids = {}
        
        print("\n[1] Creating multiple framework containers...")
        for framework, port in frameworks:
            try:
                print(f"\n  Creating {framework.value} container...")
                container_id = manager.create_container(
                    framework=framework,
                    gpu_ids=None,  # Share all GPUs
                    api_port=port,
                )
                manager.start_container(container_id)
                container_ids[framework] = container_id
                print(f"  ✓ {framework.value}: {container_id[:12]}")
            except Exception as e:
                print(f"  ✗ Failed to create {framework.value}: {e}")
        
        # Setup communication between containers
        print("\n[2] Setting up inter-container communication...")
        if FrameworkType.LEROBOT in container_ids and FrameworkType.GROOT_N15 in container_ids:
            comm_config = manager.setup_communication(
                source_framework=FrameworkType.LEROBOT,
                target_framework=FrameworkType.GROOT_N15,
                protocol=CommunicationProtocol.HTTP,
            )
            print(f"  LeRobot -> GR00T endpoint: {comm_config['endpoint']}")
        
        # List all running containers
        print("\n[3] Running containers:")
        containers = manager.list_containers()
        for container in containers:
            print(f"  - {container['framework']:15s} | "
                  f"Status: {container['status']:10s} | "
                  f"Port: {container['ports']}")
        
        # Monitor all containers
        print("\n[4] Resource monitoring:")
        for framework, container_id in container_ids.items():
            try:
                resources = manager.monitor_resources(container_id)
                print(f"  {framework.value:15s} | "
                      f"CPU: {resources.get('cpu_percent', 0):5.1f}% | "
                      f"Memory: {resources.get('memory_percent', 0):5.1f}%")
            except Exception as e:
                print(f"  {framework.value:15s} | Error: {e}")
        
        # Cleanup
        print("\n[5] Cleaning up...")
        for framework, container_id in container_ids.items():
            manager.stop_container(container_id, timeout=5)
            manager.remove_container(container_id, force=True)
            print(f"  ✓ Removed {framework.value}")
        
    finally:
        manager.cleanup()


def example_process_management():
    """Example: Starting and monitoring framework processes."""
    print("\n" + "="*80)
    print("Example 3: Process Management")
    print("="*80)
    
    manager = DockerManager()
    
    try:
        # Create and start container
        print("\n[1] Setting up LeRobot container...")
        container_id = manager.create_container(
            framework=FrameworkType.LEROBOT,
            api_port=8000,
        )
        manager.start_container(container_id)
        
        # Start inference server process
        print("\n[2] Starting LeRobot inference server...")
        process_config = {
            'mode': 'inference',
            'model_path': '/workspace/shared_data/models/lerobot_policy',
            'port': 8000,
        }
        
        try:
            process_info = manager.start_framework_process(
                container_id=container_id,
                framework=FrameworkType.LEROBOT,
                config=process_config,
            )
            print(f"  Process started with PID: {process_info['pid']}")
            print(f"  Log file: {process_info['log_file']}")
        except Exception as e:
            print(f"  Failed to start process: {e}")
        
        # Monitor process
        print("\n[3] Monitoring process...")
        if 'pid' in locals():
            import time
            for i in range(3):
                time.sleep(2)
                status = manager.monitor_process(container_id, process_info['pid'])
                print(f"  [{i+1}] Status: {status.get('status')}, "
                      f"CPU: {status.get('cpu_percent', 'N/A')}%, "
                      f"Elapsed: {status.get('elapsed_time', 'N/A')}")
        
        # Check logs
        print("\n[4] Checking process logs...")
        logs = manager.exec_command(
            container_id,
            "tail -n 20 /tmp/lerobot.log"
        )
        print("  Recent logs:")
        for line in logs.split('\n')[-5:]:
            if line.strip():
                print(f"    {line}")
        
        # Cleanup
        print("\n[5] Cleaning up...")
        manager.stop_container(container_id)
        manager.remove_container(container_id, force=True)
        
    finally:
        manager.cleanup()


def example_resource_management():
    """Example: GPU and memory resource management."""
    print("\n" + "="*80)
    print("Example 4: Resource Management")
    print("="*80)
    
    manager = DockerManager()
    
    try:
        # Create container with specific GPU allocation
        print("\n[1] Creating container with GPU 0...")
        container_id = manager.create_container(
            framework=FrameworkType.LEROBOT,
            gpu_ids=[0],  # Only GPU 0
            memory_limit='4g',  # 4GB memory limit
        )
        manager.start_container(container_id)
        
        # Update memory limits
        print("\n[2] Updating memory limits...")
        manager.set_memory_limits(container_id, memory_limit='8g')
        print("  Memory limit updated to 8GB")
        
        # Monitor resources in real-time
        print("\n[3] Monitoring resources (5 iterations)...")
        import time
        for i in range(5):
            resources = manager.monitor_resources(container_id)
            print(f"  [{i+1}] "
                  f"CPU: {resources.get('cpu_percent', 0):5.1f}% | "
                  f"Memory: {resources.get('memory_usage', 0) / (1024**3):.2f}GB / "
                  f"{resources.get('memory_limit', 0) / (1024**3):.2f}GB "
                  f"({resources.get('memory_percent', 0):5.1f}%) | "
                  f"Network RX: {resources.get('network_rx_bytes', 0) / (1024**2):.2f}MB")
            time.sleep(1)
        
        # List images
        print("\n[4] Available images:")
        images = manager.list_images()
        for img in images[:5]:  # Show first 5
            tags = ', '.join(img['tags']) if img['tags'] else 'none'
            size_mb = img['size'] / (1024**2)
            print(f"  - {tags}: {size_mb:.1f}MB")
        
        # Cleanup
        print("\n[5] Cleaning up...")
        manager.stop_container(container_id)
        manager.remove_container(container_id, force=True)
        
    finally:
        manager.cleanup()


def example_communication_patterns():
    """Example: Different communication patterns between containers."""
    print("\n" + "="*80)
    print("Example 5: Communication Patterns")
    print("="*80)
    
    manager = DockerManager()
    
    try:
        # Create two containers
        print("\n[1] Creating LeRobot and GR00T containers...")
        
        lerobot_id = manager.create_container(
            framework=FrameworkType.LEROBOT,
            api_port=8000,
        )
        manager.start_container(lerobot_id)
        
        groot_id = manager.create_container(
            framework=FrameworkType.GROOT_N15,
            api_port=8001,
        )
        manager.start_container(groot_id)
        
        # Get container IPs
        lerobot_ip = manager.get_container_ip(lerobot_id)
        groot_ip = manager.get_container_ip(groot_id)
        print(f"  LeRobot IP: {lerobot_ip}")
        print(f"  GR00T IP: {groot_ip}")
        
        # Setup different communication protocols
        print("\n[2] Setting up communication channels...")
        
        # HTTP communication
        http_config = manager.setup_communication(
            source_framework=FrameworkType.LEROBOT,
            target_framework=FrameworkType.GROOT_N15,
            protocol=CommunicationProtocol.HTTP,
        )
        print(f"  HTTP endpoint: {http_config['endpoint']}")
        
        # gRPC communication
        grpc_config = manager.setup_communication(
            source_framework=FrameworkType.LEROBOT,
            target_framework=FrameworkType.GROOT_N15,
            protocol=CommunicationProtocol.GRPC,
        )
        print(f"  gRPC endpoint: {grpc_config['endpoint']}")
        
        # ZMQ communication
        zmq_config = manager.setup_communication(
            source_framework=FrameworkType.LEROBOT,
            target_framework=FrameworkType.GROOT_N15,
            protocol=CommunicationProtocol.ZMQ,
        )
        print(f"  ZMQ endpoint: {zmq_config['endpoint']}")
        
        # Test connectivity
        print("\n[3] Testing connectivity...")
        result = manager.exec_command(
            lerobot_id,
            f"ping -c 3 {groot_ip}"
        )
        if "3 received" in result:
            print("  ✓ Network connectivity verified")
        else:
            print("  ✗ Network connectivity issue")
        
        # Cleanup
        print("\n[4] Cleaning up...")
        manager.stop_container(lerobot_id)
        manager.stop_container(groot_id)
        manager.remove_container(lerobot_id, force=True)
        manager.remove_container(groot_id, force=True)
        
    finally:
        manager.cleanup()


def main():
    """Run all examples."""
    setup_logging()
    
    examples = [
        ("Basic Usage", example_basic_usage),
        ("Multi-Framework", example_multi_framework),
        ("Process Management", example_process_management),
        ("Resource Management", example_resource_management),
        ("Communication Patterns", example_communication_patterns),
    ]
    
    print("\n" + "="*80)
    print("Docker Manager Examples for Physical AI Tools")
    print("="*80)
    print("\nAvailable examples:")
    for i, (name, _) in enumerate(examples, 1):
        print(f"  {i}. {name}")
    print(f"  {len(examples) + 1}. Run all examples")
    print("  0. Exit")
    
    try:
        choice = input("\nSelect example to run (0-{}): ".format(len(examples) + 1))
        choice = int(choice)
        
        if choice == 0:
            print("Exiting...")
            return
        elif choice == len(examples) + 1:
            print("\nRunning all examples...")
            for name, func in examples:
                try:
                    func()
                except KeyboardInterrupt:
                    print("\n\nInterrupted by user")
                    break
                except Exception as e:
                    print(f"\n\nExample '{name}' failed: {e}")
                    import traceback
                    traceback.print_exc()
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
