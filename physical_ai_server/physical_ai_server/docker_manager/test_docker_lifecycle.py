#!/usr/bin/env python3
"""
Docker Manager Lifecycle Test Suite

Comprehensive test for all Docker Manager lifecycle operations:
1. Image Management (build, pull, list, remove)
2. Container Lifecycle (create, start, stop, restart, remove)
3. Process Management (exec, logs, stats)
4. Network Management (create, connect, disconnect, remove)
5. Resource Management (GPU, memory, CPU limits)
6. Volume Management (mount, unmount)

Usage:
    python test_docker_lifecycle.py [--test TEST_NAME]
    
Available tests:
    all              - Run all tests (default)
    image            - Image management tests
    container        - Container lifecycle tests
    process          - Process management tests
    network          - Network management tests
    resource         - Resource management tests
    volume           - Volume management tests
"""

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Optional

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from physical_ai_server.docker_manager import (
    DockerManager,
    FrameworkType,
)


class Colors:
    """Terminal colors for better output."""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class LifecycleTestSuite:
    """Docker Manager lifecycle test suite."""
    
    def __init__(self):
        self.manager = DockerManager(
            shared_volume_path=Path.home() / '.cache/physical_ai_tools'
        )
        self.test_results = {}
        self.test_container_id: Optional[str] = None
        self.test_network_name = 'test_physical_ai_network'
        
    def print_header(self, text: str):
        """Print test section header."""
        print(f"\n{Colors.HEADER}{Colors.BOLD}{'='*80}{Colors.ENDC}")
        print(f"{Colors.HEADER}{Colors.BOLD}{text:^80}{Colors.ENDC}")
        print(f"{Colors.HEADER}{Colors.BOLD}{'='*80}{Colors.ENDC}\n")
        
    def print_test(self, name: str):
        """Print test name."""
        print(f"{Colors.OKBLUE}[TEST] {name}{Colors.ENDC}")
        
    def print_success(self, message: str):
        """Print success message."""
        print(f"{Colors.OKGREEN}  ✓ {message}{Colors.ENDC}")
        
    def print_fail(self, message: str):
        """Print failure message."""
        print(f"{Colors.FAIL}  ✗ {message}{Colors.ENDC}")
        
    def print_info(self, message: str):
        """Print info message."""
        print(f"{Colors.OKCYAN}  ℹ {message}{Colors.ENDC}")
        
    def print_warning(self, message: str):
        """Print warning message."""
        print(f"{Colors.WARNING}  ⚠ {message}{Colors.ENDC}")
        
    def record_result(self, test_name: str, passed: bool, message: str = ""):
        """Record test result."""
        self.test_results[test_name] = {
            'passed': passed,
            'message': message
        }
        if passed:
            self.print_success(f"PASSED: {message}")
        else:
            self.print_fail(f"FAILED: {message}")
            
    # =========================================================================
    # 1. IMAGE MANAGEMENT TESTS
    # =========================================================================
    
    def test_image_management(self):
        """Test image build, pull, list, and remove operations."""
        self.print_header("1. IMAGE MANAGEMENT TESTS")
        
        # Test 1.1: List images
        self.print_test("1.1 List Docker images")
        try:
            images = self.manager.client.images.list()
            self.print_info(f"Found {len(images)} images")
            for img in images[:3]:  # Show first 3
                tags = img.tags if img.tags else ['<none>']
                self.print_info(f"  - {tags[0]}")
            self.record_result("list_images", True, f"Listed {len(images)} images")
        except Exception as e:
            self.record_result("list_images", False, str(e))
            
        # Test 1.2: Check if GR00T image exists
        self.print_test("1.2 Check GR00T image existence")
        try:
            images = self.manager.client.images.list(name='robotis/groot_n15')
            if images:
                self.record_result("check_groot_image", True, f"Found {len(images)} GR00T image(s)")
            else:
                self.record_result("check_groot_image", False, "GR00T image not found")
        except Exception as e:
            self.record_result("check_groot_image", False, str(e))
            
        # Test 1.3: Pull a small test image (alpine)
        self.print_test("1.3 Pull test image (alpine:latest)")
        try:
            self.print_info("Pulling alpine:latest...")
            image = self.manager.client.images.pull('alpine', tag='latest')
            self.record_result("pull_image", True, f"Pulled {image.tags[0]}")
        except Exception as e:
            self.record_result("pull_image", False, str(e))
            
    # =========================================================================
    # 2. CONTAINER LIFECYCLE TESTS
    # =========================================================================
    
    def test_container_lifecycle(self):
        """Test container create, start, stop, restart, remove operations."""
        self.print_header("2. CONTAINER LIFECYCLE TESTS")
        
        # Test 2.1: Create container
        self.print_test("2.1 Create container (alpine)")
        try:
            container = self.manager.client.containers.create(
                'alpine:latest',
                name='test_lifecycle_container',
                command='sleep 300',
                detach=True,
            )
            self.test_container_id = container.id
            self.record_result("create_container", True, f"Created {container.short_id}")
        except Exception as e:
            self.record_result("create_container", False, str(e))
            return  # Can't continue without container
            
        # Test 2.2: Start container
        self.print_test("2.2 Start container")
        try:
            container = self.manager.client.containers.get(self.test_container_id)
            container.start()
            time.sleep(1)
            container.reload()
            if container.status == 'running':
                self.record_result("start_container", True, f"Status: {container.status}")
            else:
                self.record_result("start_container", False, f"Status: {container.status}")
        except Exception as e:
            self.record_result("start_container", False, str(e))
            
        # Test 2.3: Inspect container
        self.print_test("2.3 Inspect container")
        try:
            container = self.manager.client.containers.get(self.test_container_id)
            attrs = container.attrs
            self.print_info(f"Name: {attrs['Name']}")
            self.print_info(f"Status: {attrs['State']['Status']}")
            self.print_info(f"Created: {attrs['Created'][:19]}")
            self.record_result("inspect_container", True, "Retrieved container info")
        except Exception as e:
            self.record_result("inspect_container", False, str(e))
            
        # Test 2.4: List containers
        self.print_test("2.4 List containers")
        try:
            containers = self.manager.client.containers.list(all=True)
            self.print_info(f"Found {len(containers)} container(s)")
            for c in containers[:5]:  # Show first 5
                self.print_info(f"  - {c.name}: {c.status}")
            self.record_result("list_containers", True, f"Listed {len(containers)} containers")
        except Exception as e:
            self.record_result("list_containers", False, str(e))
            
        # Test 2.5: Restart container
        self.print_test("2.5 Restart container")
        try:
            container = self.manager.client.containers.get(self.test_container_id)
            container.restart(timeout=5)
            time.sleep(1)
            container.reload()
            if container.status == 'running':
                self.record_result("restart_container", True, f"Status: {container.status}")
            else:
                self.record_result("restart_container", False, f"Status: {container.status}")
        except Exception as e:
            self.record_result("restart_container", False, str(e))
            
        # Test 2.6: Stop container
        self.print_test("2.6 Stop container")
        try:
            container = self.manager.client.containers.get(self.test_container_id)
            container.stop(timeout=5)
            time.sleep(1)
            container.reload()
            if container.status == 'exited':
                self.record_result("stop_container", True, f"Status: {container.status}")
            else:
                self.record_result("stop_container", False, f"Status: {container.status}")
        except Exception as e:
            self.record_result("stop_container", False, str(e))
            
        # Test 2.7: Remove container
        self.print_test("2.7 Remove container")
        try:
            container = self.manager.client.containers.get(self.test_container_id)
            container.remove(force=True)
            self.record_result("remove_container", True, "Container removed")
            self.test_container_id = None
        except Exception as e:
            self.record_result("remove_container", False, str(e))
            
    # =========================================================================
    # 3. PROCESS MANAGEMENT TESTS
    # =========================================================================
    
    def test_process_management(self):
        """Test exec, logs, and stats operations."""
        self.print_header("3. PROCESS MANAGEMENT TESTS")
        
        # Create a test container
        self.print_test("3.0 Setup: Create test container")
        try:
            container = self.manager.client.containers.run(
                'alpine:latest',
                name='test_process_container',
                command='sh -c "while true; do echo Hello; sleep 1; done"',
                detach=True,
            )
            self.test_container_id = container.id
            time.sleep(2)  # Let it generate some logs
            self.print_success("Test container created")
        except Exception as e:
            self.print_fail(f"Setup failed: {e}")
            return
            
        # Test 3.1: Execute command in container
        self.print_test("3.1 Execute command (ls /)")
        try:
            container = self.manager.client.containers.get(self.test_container_id)
            result = container.exec_run('ls /')
            if result.exit_code == 0:
                output = result.output.decode('utf-8').strip().split('\n')
                self.print_info(f"Found {len(output)} directories/files")
                self.record_result("exec_command", True, f"Exit code: {result.exit_code}")
            else:
                self.record_result("exec_command", False, f"Exit code: {result.exit_code}")
        except Exception as e:
            self.record_result("exec_command", False, str(e))
            
        # Test 3.2: Get container logs
        self.print_test("3.2 Get container logs")
        try:
            container = self.manager.client.containers.get(self.test_container_id)
            logs = container.logs(tail=5).decode('utf-8')
            lines = logs.strip().split('\n')
            self.print_info(f"Retrieved {len(lines)} log lines")
            for line in lines[:3]:
                self.print_info(f"  Log: {line}")
            self.record_result("get_logs", True, f"Retrieved {len(lines)} lines")
        except Exception as e:
            self.record_result("get_logs", False, str(e))
            
        # Test 3.3: Get container stats
        self.print_test("3.3 Get container stats")
        try:
            container = self.manager.client.containers.get(self.test_container_id)
            stats = container.stats(stream=False)
            
            # Parse stats
            memory_usage = stats['memory_stats'].get('usage', 0) / (1024 * 1024)  # MB
            cpu_delta = stats['cpu_stats']['cpu_usage']['total_usage'] - \
                       stats['precpu_stats']['cpu_usage']['total_usage']
            system_delta = stats['cpu_stats']['system_cpu_usage'] - \
                          stats['precpu_stats']['system_cpu_usage']
            cpu_percent = (cpu_delta / system_delta) * 100.0 if system_delta > 0 else 0.0
            
            self.print_info(f"Memory: {memory_usage:.2f} MB")
            self.print_info(f"CPU: {cpu_percent:.2f}%")
            self.record_result("get_stats", True, "Retrieved container stats")
        except Exception as e:
            self.record_result("get_stats", False, str(e))
            
        # Cleanup
        self.print_test("3.9 Cleanup: Remove test container")
        try:
            container = self.manager.client.containers.get(self.test_container_id)
            container.remove(force=True)
            self.test_container_id = None
            self.print_success("Cleanup complete")
        except Exception as e:
            self.print_fail(f"Cleanup failed: {e}")
            
    # =========================================================================
    # 4. NETWORK MANAGEMENT TESTS
    # =========================================================================
    
    def test_network_management(self):
        """Test network create, connect, disconnect, remove operations."""
        self.print_header("4. NETWORK MANAGEMENT TESTS")
        
        # Test 4.1: List networks
        self.print_test("4.1 List Docker networks")
        try:
            networks = self.manager.client.networks.list()
            self.print_info(f"Found {len(networks)} network(s)")
            for net in networks:
                self.print_info(f"  - {net.name}: {net.attrs['Driver']}")
            self.record_result("list_networks", True, f"Listed {len(networks)} networks")
        except Exception as e:
            self.record_result("list_networks", False, str(e))
            
        # Test 4.2: Create network
        self.print_test("4.2 Create custom network")
        try:
            # Remove if exists
            try:
                existing = self.manager.client.networks.get(self.test_network_name)
                existing.remove()
                self.print_info("Removed existing network")
            except:
                pass
                
            network = self.manager.client.networks.create(
                self.test_network_name,
                driver='bridge',
            )
            self.print_info(f"Network ID: {network.short_id}")
            self.record_result("create_network", True, f"Created {self.test_network_name}")
        except Exception as e:
            self.record_result("create_network", False, str(e))
            return
            
        # Test 4.3: Connect container to network
        self.print_test("4.3 Connect container to network")
        try:
            # Create test container
            container = self.manager.client.containers.create(
                'alpine:latest',
                name='test_network_container',
                command='sleep 30',
            )
            self.test_container_id = container.id
            
            # Connect to network
            network = self.manager.client.networks.get(self.test_network_name)
            network.connect(container)
            
            # Verify connection
            network.reload()
            connected_containers = network.attrs['Containers']
            if container.id in connected_containers:
                self.record_result("connect_network", True, "Container connected")
            else:
                self.record_result("connect_network", False, "Container not in network")
        except Exception as e:
            self.record_result("connect_network", False, str(e))
            
        # Test 4.4: Disconnect container from network
        self.print_test("4.4 Disconnect container from network")
        try:
            container = self.manager.client.containers.get(self.test_container_id)
            network = self.manager.client.networks.get(self.test_network_name)
            network.disconnect(container)
            
            # Verify disconnection
            network.reload()
            connected_containers = network.attrs['Containers']
            if container.id not in connected_containers:
                self.record_result("disconnect_network", True, "Container disconnected")
            else:
                self.record_result("disconnect_network", False, "Container still in network")
                
            # Cleanup container
            container.remove(force=True)
            self.test_container_id = None
        except Exception as e:
            self.record_result("disconnect_network", False, str(e))
            
        # Test 4.5: Remove network
        self.print_test("4.5 Remove custom network")
        try:
            network = self.manager.client.networks.get(self.test_network_name)
            network.remove()
            self.record_result("remove_network", True, f"Removed {self.test_network_name}")
        except Exception as e:
            self.record_result("remove_network", False, str(e))
            
    # =========================================================================
    # 5. RESOURCE MANAGEMENT TESTS
    # =========================================================================
    
    def test_resource_management(self):
        """Test GPU, memory, and CPU resource management."""
        self.print_header("5. RESOURCE MANAGEMENT TESTS")
        
        # Test 5.1: Check GPU availability
        self.print_test("5.1 Check GPU availability")
        try:
            has_gpu = self.manager._has_nvidia_runtime()
            if has_gpu:
                self.print_info("NVIDIA GPU runtime detected")
                self.record_result("check_gpu", True, "GPU available")
            else:
                self.print_warning("NVIDIA GPU runtime not detected")
                self.record_result("check_gpu", True, "GPU not available (expected)")
        except Exception as e:
            self.record_result("check_gpu", False, str(e))
            
        # Test 5.2: Create container with memory limit
        self.print_test("5.2 Create container with memory limit (256MB)")
        try:
            container = self.manager.client.containers.create(
                'alpine:latest',
                name='test_memory_container',
                command='sleep 30',
                mem_limit='256m',
            )
            self.test_container_id = container.id
            
            # Verify memory limit
            container.reload()
            memory_limit = container.attrs['HostConfig']['Memory']
            memory_mb = memory_limit / (1024 * 1024)
            self.print_info(f"Memory limit: {memory_mb:.0f} MB")
            
            if memory_mb == 256:
                self.record_result("memory_limit", True, f"Set to {memory_mb:.0f} MB")
            else:
                self.record_result("memory_limit", False, f"Expected 256 MB, got {memory_mb:.0f} MB")
                
            # Cleanup
            container.remove(force=True)
            self.test_container_id = None
        except Exception as e:
            self.record_result("memory_limit", False, str(e))
            
        # Test 5.3: Create container with CPU limit
        self.print_test("5.3 Create container with CPU limit")
        try:
            container = self.manager.client.containers.create(
                'alpine:latest',
                name='test_cpu_container',
                command='sleep 30',
                nano_cpus=int(1e9),  # 1 CPU
            )
            self.test_container_id = container.id
            
            # Verify CPU limit
            container.reload()
            nano_cpus = container.attrs['HostConfig']['NanoCpus']
            cpus = nano_cpus / 1e9
            self.print_info(f"CPU limit: {cpus:.1f} CPUs")
            
            if cpus == 1.0:
                self.record_result("cpu_limit", True, f"Set to {cpus:.1f} CPU")
            else:
                self.record_result("cpu_limit", False, f"Expected 1 CPU, got {cpus:.1f}")
                
            # Cleanup
            container.remove(force=True)
            self.test_container_id = None
        except Exception as e:
            self.record_result("cpu_limit", False, str(e))
            
    # =========================================================================
    # 6. VOLUME MANAGEMENT TESTS
    # =========================================================================
    
    def test_volume_management(self):
        """Test volume mount and unmount operations."""
        self.print_header("6. VOLUME MANAGEMENT TESTS")
        
        # Test 6.1: Create container with bind mount
        self.print_test("6.1 Create container with bind mount")
        try:
            from docker.types import Mount
            
            # Create a test directory
            test_dir = Path.home() / '.cache/physical_ai_tools/test_mount'
            test_dir.mkdir(parents=True, exist_ok=True)
            test_file = test_dir / 'test.txt'
            test_file.write_text('Hello from host!')
            
            # Create container with mount
            mount = Mount(
                target='/mnt/test',
                source=str(test_dir),
                type='bind',
            )
            
            container = self.manager.client.containers.create(
                'alpine:latest',
                name='test_volume_container',
                command='sleep 30',
                mounts=[mount],
            )
            self.test_container_id = container.id
            container.start()
            time.sleep(1)
            
            # Verify mount by reading file
            result = container.exec_run('cat /mnt/test/test.txt')
            content = result.output.decode('utf-8').strip()
            
            if content == 'Hello from host!':
                self.record_result("bind_mount", True, "File accessible from container")
            else:
                self.record_result("bind_mount", False, f"Unexpected content: {content}")
                
            # Cleanup
            container.stop(timeout=5)
            container.remove(force=True)
            self.test_container_id = None
            test_file.unlink()
            
        except Exception as e:
            self.record_result("bind_mount", False, str(e))
            
        # Test 6.2: Create and use Docker volume
        self.print_test("6.2 Create and use Docker volume")
        try:
            # Create volume
            volume = self.manager.client.volumes.create(name='test_volume')
            self.print_info(f"Volume created: {volume.name}")
            
            # Create container using volume
            container = self.manager.client.containers.run(
                'alpine:latest',
                name='test_volume_user',
                command='sh -c "echo test > /data/file.txt && cat /data/file.txt"',
                volumes={volume.name: {'bind': '/data', 'mode': 'rw'}},
                detach=True,
            )
            
            # Wait and check logs
            time.sleep(2)
            logs = container.logs().decode('utf-8').strip()
            
            if 'test' in logs:
                self.record_result("docker_volume", True, "Volume accessible")
            else:
                self.record_result("docker_volume", False, f"Unexpected output: {logs}")
                
            # Cleanup
            container.remove(force=True)
            volume.remove(force=True)
            
        except Exception as e:
            self.record_result("docker_volume", False, str(e))
            
    # =========================================================================
    # TEST RUNNER
    # =========================================================================
    
    def run_all_tests(self):
        """Run all test suites."""
        self.test_image_management()
        self.test_container_lifecycle()
        self.test_process_management()
        self.test_network_management()
        self.test_resource_management()
        self.test_volume_management()
        
    def run_test(self, test_name: str):
        """Run a specific test suite."""
        test_map = {
            'image': self.test_image_management,
            'container': self.test_container_lifecycle,
            'process': self.test_process_management,
            'network': self.test_network_management,
            'resource': self.test_resource_management,
            'volume': self.test_volume_management,
            'all': self.run_all_tests,
        }
        
        if test_name not in test_map:
            print(f"{Colors.FAIL}Unknown test: {test_name}{Colors.ENDC}")
            print(f"Available tests: {', '.join(test_map.keys())}")
            return
            
        test_map[test_name]()
        
    def print_summary(self):
        """Print test summary."""
        self.print_header("TEST SUMMARY")
        
        passed = sum(1 for r in self.test_results.values() if r['passed'])
        failed = len(self.test_results) - passed
        total = len(self.test_results)
        
        print(f"{Colors.BOLD}Total Tests: {total}{Colors.ENDC}")
        print(f"{Colors.OKGREEN}Passed: {passed}{Colors.ENDC}")
        print(f"{Colors.FAIL}Failed: {failed}{Colors.ENDC}")
        print(f"\nPass Rate: {(passed/total*100):.1f}%\n")
        
        if failed > 0:
            print(f"{Colors.FAIL}{Colors.BOLD}Failed Tests:{Colors.ENDC}")
            for name, result in self.test_results.items():
                if not result['passed']:
                    print(f"{Colors.FAIL}  ✗ {name}: {result['message']}{Colors.ENDC}")
                    
    def cleanup(self):
        """Cleanup any remaining test resources."""
        self.print_header("CLEANUP")
        
        # Remove test container if exists
        if self.test_container_id:
            try:
                container = self.manager.client.containers.get(self.test_container_id)
                container.remove(force=True)
                print(f"{Colors.OKGREEN}  ✓ Removed test container{Colors.ENDC}")
            except:
                pass
                
        # Remove test network if exists
        try:
            network = self.manager.client.networks.get(self.test_network_name)
            network.remove()
            print(f"{Colors.OKGREEN}  ✓ Removed test network{Colors.ENDC}")
        except:
            pass
            
        # Remove any containers with 'test_' prefix
        try:
            containers = self.manager.client.containers.list(
                all=True,
                filters={'name': 'test_'}
            )
            for container in containers:
                try:
                    container.remove(force=True)
                    print(f"{Colors.OKGREEN}  ✓ Removed {container.name}{Colors.ENDC}")
                except:
                    pass
        except:
            pass


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Docker Manager Lifecycle Test Suite',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Available tests:
  all              - Run all tests (default)
  image            - Image management tests
  container        - Container lifecycle tests
  process          - Process management tests
  network          - Network management tests
  resource         - Resource management tests
  volume           - Volume management tests

Examples:
  python test_docker_lifecycle.py                  # Run all tests
  python test_docker_lifecycle.py --test container # Run only container tests
  python test_docker_lifecycle.py --test process   # Run only process tests
        """
    )
    parser.add_argument(
        '--test',
        type=str,
        default='all',
        help='Specific test to run (default: all)'
    )
    
    args = parser.parse_args()
    
    # Setup logging
    logging.basicConfig(
        level=logging.WARNING,  # Reduce noise
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Run tests
    suite = LifecycleTestSuite()
    
    try:
        suite.run_test(args.test)
    except KeyboardInterrupt:
        print(f"\n{Colors.WARNING}Tests interrupted by user{Colors.ENDC}")
    except Exception as e:
        print(f"\n{Colors.FAIL}Unexpected error: {e}{Colors.ENDC}")
        import traceback
        traceback.print_exc()
    finally:
        suite.cleanup()
        suite.print_summary()


if __name__ == '__main__':
    main()
