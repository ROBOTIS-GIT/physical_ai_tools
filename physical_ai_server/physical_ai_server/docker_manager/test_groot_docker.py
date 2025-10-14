#!/usr/bin/env python3
"""
GR00T N1.5 Docker Integration Test

This script demonstrates how to use Docker Manager to run GR00T N1.5
in a container and communicate with it via ZMQ.

Usage:
    python test_groot_docker.py [--build] [--test-inference]
"""

import argparse
import logging
import sys
import time
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from physical_ai_server.docker_manager import (
    DockerManager,
    FrameworkType,
    ZMQClient,
    ZMQClientPool,
)


def setup_logging():
    """Setup logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


def test_groot_docker(
    build_image: bool = False,
    test_inference: bool = True,
    use_gpu: bool = True,
    debug_only: bool = False,
):
    """
    Test GR00T N1.5 Docker integration.
    
    Args:
        build_image: Whether to build Docker image first
        test_inference: Whether to test inference
    """
    print("\n" + "="*80)
    print("GR00T N1.5 Docker Integration Test")
    print("="*80)
    
    # Paths
    groot_path = Path('/home/dongyun/ros2_ws/src/physical_ai_tools/Isaac-GR00T')
    checkpoint_path = Path.home() / 'ext_data_storage/gr00t_n1_5_data'
    
    # Create checkpoint directory if it doesn't exist
    try:
        checkpoint_path.mkdir(parents=True, exist_ok=True)
        print(f"Checkpoint path: {checkpoint_path}")
    except Exception as e:
        print(f"Warning: Could not create checkpoint directory: {e}")
        checkpoint_path = None
    
    # Initialize Docker Manager
    print("\n[Step 1] Initializing Docker Manager...")
    manager = DockerManager(
        shared_volume_path=Path.home() / '.cache/physical_ai_tools',
    )
    
    # Cleanup any existing containers with the same name
    print("  Checking for existing GR00T containers...")
    try:
        existing_containers = manager.client.containers.list(
            all=True,
            filters={'name': 'physical_ai_groot_n15'}
        )
        if existing_containers:
            print(f"  Found {len(existing_containers)} existing container(s), removing...")
            for container in existing_containers:
                try:
                    container.stop(timeout=5)
                    print(f"    Stopped: {container.short_id}")
                except:
                    pass
                try:
                    container.remove(force=True)
                    print(f"    Removed: {container.short_id}")
                except Exception as e:
                    print(f"    Warning: Could not remove {container.short_id}: {e}")
    except Exception as e:
        print(f"  Warning: Could not check for existing containers: {e}")
    
    try:
        # Build image if requested
        if build_image:
            print("\n[Step 2] Building GR00T Docker image...")
            print(f"  Dockerfile: {groot_path / 'Dockerfile'}")
            
            try:
                image = manager.build_image(
                    framework=FrameworkType.GROOT_N15,
                    dockerfile_path=groot_path / 'Dockerfile',
                    tag='robotis/groot_n15:latest',
                )
                print(f"  ✓ Image built: {image.tags}")
                
                # If debug_only mode, stop here
                if debug_only:
                    print("\n" + "="*80)
                    print("✓ Build complete! Use this command to debug:")
                    print("="*80)
                    print(f"docker run --gpus all -it --rm --name groot_debug \\")
                    print(f"  --shm-size=64g --network host \\")
                    print(f"  -v {groot_path}:/workspace \\")
                    if checkpoint_path and checkpoint_path.exists():
                        print(f"  -v {checkpoint_path}:/workspace/checkpoints \\")
                    print(f"  robotis/groot_n15:latest /bin/bash")
                    print("\nInside container, run:")
                    print("python scripts/zmq_inference_server.py --port 5556 --model_path nvidia/GR00T-N1.5-3B --embodiment_tag default --device cuda")
                    print("="*80)
                    return
                    
            except Exception as e:
                print(f"  ✗ Build failed: {e}")
                return
        else:
            print("\n[Step 2] Skipping image build (use --build to build)")
        
        # Create container
        print("\n[Step 3] Creating GR00T container...")
        
        # Custom container config matching your CLI command
        container_config = {
            'shm_size': '64g',  # --shm-size=64g
            'network_mode': 'host',  # --network host
            # remove parameter is not supported in containers.create()
            'tty': True,  # -it
            'stdin_open': True,  # -it
        }
        
        # Setup volumes
        volumes = []
        
        # Mount Isaac-GR00T directory
        from docker.types import Mount
        volumes.append(
            Mount(
                target='/workspace',
                source=str(groot_path),
                type='bind',
            )
        )
        
        # Mount checkpoint directory (if available and exists)
        if checkpoint_path and checkpoint_path.exists():
            volumes.append(
                Mount(
                    target='/workspace/checkpoints',
                    source=str(checkpoint_path),
                    type='bind',
                )
            )
            print(f"  Mounting checkpoint path: {checkpoint_path}")
        else:
            print("  Checkpoint path not available - will download model from HuggingFace")
        
        container_config['mounts'] = volumes
        
        # Check if we should use GPU
        # Use empty list to explicitly disable GPU
        gpu_config = None if use_gpu else []
        
        container_id = manager.create_container(
            framework=FrameworkType.GROOT_N15,
            config=container_config,
            gpu_ids=gpu_config,  # None=auto-detect, []=no GPU
            api_port=5556,  # ZMQ port
        )
        
        print(f"  ✓ Container created: {container_id[:12]}")
        
        # Start container
        print("\n[Step 4] Starting container...")
        manager.start_container(container_id)
        print("  ✓ Container started")
        
        # Start ZMQ server process
        print("\n[Step 5] Starting GR00T ZMQ server...")
        
        process_config = {
            'mode': 'inference',
            'port': 5556,
            'model_path': 'nvidia/GR00T-N1.5-3B',
            'embodiment_tag': 'default',
            'device': 'cuda',
        }
        
        try:
            process_info = manager.start_framework_process(
                container_id=container_id,
                framework=FrameworkType.GROOT_N15,
                config=process_config,
            )
            print(f"  ✓ ZMQ server started (PID: {process_info['pid']})")
            print(f"  Log file: {process_info['log_file']}")
        except Exception as e:
            print(f"  ✗ Failed to start server: {e}")
            print("  Trying to check if server is already running...")
        
        # Wait for server to be ready
        print("\n[Step 6] Waiting for server to be ready...")
        time.sleep(10)  # Give it time to load the model
        
        # Setup ZMQ client
        print("\n[Step 7] Setting up ZMQ client...")
        container_ip = manager.get_container_ip(container_id)
        
        if container_ip:
            endpoint = f'tcp://{container_ip}:5556'
            print(f"  Container IP: {container_ip}")
            print(f"  ZMQ endpoint: {endpoint}")
        else:
            # Using host network, use localhost
            endpoint = 'tcp://localhost:5556'
            print(f"  Using host network: {endpoint}")
        
        client = ZMQClient(endpoint, timeout=30000)  # 30 second timeout
        
        # Health check
        print("\n[Step 8] Performing health check...")
        if client.health_check():
            print("  ✓ Server is healthy")
        else:
            print("  ✗ Server is not responding")
            print("  Check container logs:")
            logs = manager.exec_command(container_id, "tail -n 50 /tmp/groot_n15.log")
            print(logs)
            return
        
        # Test inference
        if test_inference:
            print("\n[Step 9] Testing inference...")
            
            # Create dummy observation
            import numpy as np
            observation = {
                'image': np.random.rand(224, 224, 3).tolist(),  # Dummy image
                'robot_state': [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],  # Dummy state
                'instruction': 'Pick up the coffee bottle',  # Optional
            }
            
            print("  Sending inference request...")
            response = client.inference(observation)
            
            if response and response.get('status') == 'success':
                result = response['result']
                action = result.get('action', [])
                print(f"  ✓ Received action: {action[:5]}...")  # Show first 5 values
                print(f"  Action shape: {len(action)} dimensions")
            else:
                print(f"  ✗ Inference failed: {response}")
        else:
            print("\n[Step 9] Skipping inference test (use --test-inference)")
        
        # Monitor resources
        print("\n[Step 10] Monitoring container resources...")
        resources = manager.monitor_resources(container_id)
        print(f"  CPU: {resources.get('cpu_percent', 0):5.1f}%")
        print(f"  Memory: {resources.get('memory_usage', 0) / (1024**3):.2f}GB / "
              f"{resources.get('memory_limit', 0) / (1024**3):.2f}GB "
              f"({resources.get('memory_percent', 0):5.1f}%)")
        
        # Keep container running
        print("\n[Step 11] Container is running!")
        print("\nTo interact with the container:")
        print(f"  docker exec -it {container_id[:12]} bash")
        print("\nTo stop the container:")
        print(f"  docker stop {container_id[:12]}")
        print("\nPress Ctrl+C to stop and cleanup...")
        
        try:
            while True:
                time.sleep(10)
                # Periodic health check
                if not client.health_check():
                    print("  ⚠ Server stopped responding")
                    break
        except KeyboardInterrupt:
            print("\n\nStopping...")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        print("\n[Cleanup] Stopping container...")
        try:
            if 'client' in locals():
                client.close()
            
            if 'container_id' in locals():
                manager.stop_container(container_id, timeout=10)
                print(f"  ✓ Container stopped")
                
                # Remove container
                response = input("Remove container? [y/N]: ")
                if response.lower() == 'y':
                    manager.remove_container(container_id, force=True)
                    print("  ✓ Container removed")
                else:
                    print(f"  Container kept: {container_id[:12]}")
        except Exception as e:
            print(f"  ✗ Cleanup error: {e}")
        
        manager.cleanup()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Test GR00T N1.5 Docker Integration'
    )
    parser.add_argument(
        '--build',
        action='store_true',
        help='Build Docker image before testing'
    )
    parser.add_argument(
        '--test-inference',
        action='store_true',
        default=True,
        help='Test inference (default: True)'
    )
    parser.add_argument(
        '--no-inference',
        action='store_true',
        help='Skip inference test'
    )
    parser.add_argument(
        '--no-gpu',
        action='store_true',
        help='Run without GPU (CPU only mode)'
    )
    parser.add_argument(
        '--debug-only',
        action='store_true',
        help='Build image only and show debug command (no container start)'
    )
    
    args = parser.parse_args()
    
    setup_logging()
    
    test_inference = args.test_inference and not args.no_inference
    
    test_groot_docker(
        build_image=args.build,
        test_inference=test_inference,
        use_gpu=not args.no_gpu,
        debug_only=args.debug_only,
    )


if __name__ == '__main__':
    main()
