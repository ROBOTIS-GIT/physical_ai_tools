"""
Docker Manager for Physical AI Tools

This module manages the lifecycle of Docker containers for different Physical AI frameworks
(LeRobot, GR00T N1.5, Pi0, etc.). It handles image building, container lifecycle management,
inter-container communication, and resource allocation.

Architecture:
    Physical AI Server (Orchestrator)
    ├── Docker Manager (lifecycle management)
    ├── Communication Layer (API-based)
    ├── Resource Manager (GPU, memory allocation)
    └── Volume Manager (shared data)

Communication Strategy:
    - Primary: HTTP/gRPC (ROS2 version independent)
    - Data Sharing: Shared Docker volumes
    - Inference: ZMQ (existing implementation)
    - External Interface: ROS2 DDS (Physical AI Server only)
"""

import json
import logging
import os
import time
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Union, TYPE_CHECKING

try:
    import docker
    from docker.errors import (
        APIError,
        BuildError,
        ContainerError,
        DockerException,
        ImageNotFound,
        NotFound,
    )
    from docker.models.containers import Container
    from docker.models.images import Image
    from docker.models.networks import Network
    from docker.types import DeviceRequest, Mount

    DOCKER_AVAILABLE = True
except ImportError:
    DOCKER_AVAILABLE = False
    logging.warning("Docker SDK not available. Install with: pip install docker")
    
    # Define placeholder types for type hints when docker is not available
    if TYPE_CHECKING:
        from docker.models.containers import Container
        from docker.models.images import Image
        from docker.models.networks import Network
    else:
        Container = Any
        Image = Any
        Network = Any


class FrameworkType(Enum):
    """Supported Physical AI frameworks"""
    LEROBOT = "lerobot"
    GROOT_N15 = "groot_n15"
    PI0 = "pi0"
    ISAAC_SIM = "isaac_sim"
    ISAAC_LAB = "isaac_lab"


class ContainerStatus(Enum):
    """Container lifecycle status"""
    NOT_EXIST = "not_exist"
    CREATED = "created"
    RUNNING = "running"
    PAUSED = "paused"
    RESTARTING = "restarting"
    REMOVING = "removing"
    EXITED = "exited"
    DEAD = "dead"


class CommunicationProtocol(Enum):
    """Communication protocols between containers"""
    ZMQ = "zmq"  # Primary protocol for all container communication
    SHARED_VOLUME = "shared_volume"  # For large data files


class DockerManager:
    """
    Manages Docker containers for Physical AI frameworks.
    
    This class provides comprehensive Docker container management including:
    - Image building and pulling
    - Container lifecycle management
    - Network configuration
    - GPU and resource allocation
    - Inter-container communication
    - Process management within containers
    
    Attributes:
        client: Docker client instance
        network_name: Name of the Docker network for inter-container communication
        base_image_registry: Docker registry for base images
        shared_volume_path: Host path for shared volumes
    """

    DEFAULT_NETWORK_NAME = "physical_ai_network"
    DEFAULT_VOLUME_PATH = Path.home() / ".cache/physical_ai_tools"
    DEFAULT_API_PORT_BASE = 5555  # Base port for framework ZMQ servers
    
    # Default ZMQ ports for each framework
    FRAMEWORK_PORTS = {
        FrameworkType.LEROBOT: 5555,
        FrameworkType.GROOT_N15: 5556,
        FrameworkType.PI0: 5557,
    }

    def __init__(
        self,
        network_name: str = DEFAULT_NETWORK_NAME,
        shared_volume_path: Optional[Path] = None,
        base_image_registry: str = "robotis",
        logger: Optional[logging.Logger] = None,
    ):
        """
        Initialize Docker Manager.
        
        Args:
            network_name: Name of Docker network to create/use
            shared_volume_path: Path for shared volumes between host and containers
            base_image_registry: Docker registry prefix for images
            logger: Logger instance (creates default if None)
        
        Raises:
            DockerException: If Docker daemon is not accessible
        """
        if not DOCKER_AVAILABLE:
            raise ImportError(
                "Docker SDK not installed. Install with: pip install docker"
            )

        self.logger = logger or logging.getLogger(__name__)
        
        try:
            self.client = docker.from_env()
            # Test connection
            self.client.ping()
            self.logger.info("Connected to Docker daemon successfully")
        except DockerException as e:
            self.logger.error(f"Failed to connect to Docker daemon: {e}")
            raise

        self.network_name = network_name
        self.base_image_registry = base_image_registry
        self.shared_volume_path = shared_volume_path or self.DEFAULT_VOLUME_PATH
        
        # Ensure shared volume path exists
        self.shared_volume_path.mkdir(parents=True, exist_ok=True)
        
        # Container registry: framework_name -> container_id
        self._containers: Dict[str, str] = {}
        
        # Port mapping: framework_name -> port
        self._port_mapping: Dict[str, int] = {}
        
        # Initialize network
        self._network: Optional[Network] = None
        self._ensure_network()

    def _ensure_network(self) -> Network:
        """
        Ensure Docker network exists for inter-container communication.
        
        Returns:
            Docker network object
        """
        try:
            networks = self.client.networks.list(names=[self.network_name])
            if networks:
                self._network = networks[0]
                self.logger.info(f"Using existing network: {self.network_name}")
            else:
                self._network = self.client.networks.create(
                    name=self.network_name,
                    driver="bridge",
                    check_duplicate=True,
                )
                self.logger.info(f"Created new network: {self.network_name}")
        except APIError as e:
            self.logger.error(f"Failed to create/access network: {e}")
            raise
        
        return self._network

    # ============================================================================
    # Image Management
    # ============================================================================

    def build_image(
        self,
        framework: FrameworkType,
        dockerfile_path: Path,
        platform: Optional[str] = None,
        build_args: Optional[Dict[str, str]] = None,
        tag: Optional[str] = None,
    ) -> Image:
        """
        Build Docker image for a specific framework.
        
        Args:
            framework: Framework type to build image for
            dockerfile_path: Path to Dockerfile
            platform: Target platform (e.g., 'linux/amd64', 'linux/arm64')
            build_args: Build arguments to pass to Docker build
            tag: Custom image tag (auto-generated if None)
        
        Returns:
            Built Docker image object
        
        Raises:
            BuildError: If image build fails
        """
        if tag is None:
            tag = f"{self.base_image_registry}/{framework.value}:latest"
        
        build_args = build_args or {}
        
        self.logger.info(f"Building image for {framework.value}...")
        self.logger.info(f"  Dockerfile: {dockerfile_path}")
        self.logger.info(f"  Platform: {platform or 'default'}")
        self.logger.info(f"  Tag: {tag}")
        self.logger.info("=" * 80)
        
        try:
            # Build with streaming output
            build_result = self.client.api.build(
                path=str(dockerfile_path.parent),
                dockerfile=str(dockerfile_path.name),
                tag=tag,
                platform=platform,
                buildargs=build_args,
                rm=True,  # Remove intermediate containers
                forcerm=True,  # Always remove intermediate containers
                decode=True,  # Decode JSON response
            )
            
            # Log build output in real-time
            image_id = None
            for log in build_result:
                if 'stream' in log:
                    msg = log['stream'].rstrip()
                    if msg:  # Skip empty lines
                        print(msg)  # Print to stdout for real-time viewing
                        self.logger.info(msg)
                elif 'error' in log:
                    error_msg = log['error']
                    print(f"ERROR: {error_msg}")
                    self.logger.error(error_msg)
                    raise BuildError(error_msg, build_result)
                elif 'aux' in log:
                    # Extract image ID from auxiliary info
                    if 'ID' in log['aux']:
                        image_id = log['aux']['ID']
                elif 'status' in log:
                    status_msg = log['status']
                    if 'id' in log:
                        status_msg = f"{log['id']}: {status_msg}"
                    print(status_msg)
                    self.logger.info(status_msg)
            
            self.logger.info("=" * 80)
            self.logger.info(f"Successfully built image: {tag}")
            
            # Get the image object
            image = self.client.images.get(tag)
            return image
            
        except BuildError as e:
            self.logger.error(f"Failed to build image: {e}")
            raise

    def pull_image(
        self,
        framework: FrameworkType,
        tag: str = "latest",
    ) -> Image:
        """
        Pull Docker image from registry.
        
        Args:
            framework: Framework type to pull image for
            tag: Image tag to pull
        
        Returns:
            Pulled Docker image object
        
        Raises:
            APIError: If image pull fails
        """
        image_name = f"{self.base_image_registry}/{framework.value}:{tag}"
        
        self.logger.info(f"Pulling image: {image_name}")
        
        try:
            image = self.client.images.pull(image_name)
            self.logger.info(f"Successfully pulled image: {image_name}")
            return image
        except APIError as e:
            self.logger.error(f"Failed to pull image: {e}")
            raise

    def list_images(
        self,
        framework: Optional[FrameworkType] = None,
    ) -> List[Dict[str, Any]]:
        """
        List available Docker images.
        
        Args:
            framework: Filter by specific framework (None for all)
        
        Returns:
            List of image information dictionaries
        """
        images = self.client.images.list()
        
        result = []
        for image in images:
            # Filter by framework if specified
            if framework:
                pattern = f"{self.base_image_registry}/{framework.value}"
                if not any(pattern in tag for tag in image.tags):
                    continue
            
            image_info = {
                'id': image.short_id,
                'tags': image.tags,
                'size': image.attrs['Size'],
                'created': image.attrs['Created'],
            }
            result.append(image_info)
        
        return result

    def remove_image(
        self,
        framework: FrameworkType,
        tag: str = "latest",
        force: bool = False,
    ) -> None:
        """
        Remove Docker image.
        
        Args:
            framework: Framework type to remove image for
            tag: Image tag to remove
            force: Force removal even if containers are using it
        
        Raises:
            ImageNotFound: If image doesn't exist
        """
        image_name = f"{self.base_image_registry}/{framework.value}:{tag}"
        
        try:
            self.client.images.remove(image_name, force=force)
            self.logger.info(f"Removed image: {image_name}")
        except ImageNotFound:
            self.logger.warning(f"Image not found: {image_name}")
        except APIError as e:
            self.logger.error(f"Failed to remove image: {e}")
            raise

    # ============================================================================
    # Container Lifecycle Management
    # ============================================================================

    def create_container(
        self,
        framework: FrameworkType,
        config: Optional[Dict[str, Any]] = None,
        gpu_ids: Optional[List[int]] = None,
        memory_limit: Optional[str] = None,
        api_port: Optional[int] = None,
    ) -> str:
        """
        Create a container for a specific framework.
        
        Args:
            framework: Framework type to create container for
            config: Additional container configuration
            gpu_ids: List of GPU IDs to allocate (None for all GPUs)
            memory_limit: Memory limit (e.g., '4g', '512m')
            api_port: Port for framework API server
        
        Returns:
            Container ID
        
        Raises:
            APIError: If container creation fails
        """
        container_name = f"physical_ai_{framework.value}"
        image_name = f"{self.base_image_registry}/{framework.value}:latest"
        
        # Check if container already exists
        if container_name in self._containers:
            self.logger.warning(
                f"Container {container_name} already exists. "
                "Use start_container() to start it."
            )
            return self._containers[container_name]
        
        # Default configuration
        default_config = {
            'name': container_name,
            'image': image_name,
            'detach': True,
            'network': self.network_name,
            'tty': True,
            'stdin_open': True,
            'privileged': True,  # Needed for device access
            'environment': {
                'FRAMEWORK_TYPE': framework.value,
            },
        }
        
        # Setup volumes
        volumes = self._setup_volumes(framework)
        default_config['mounts'] = volumes
        
        # Setup GPU allocation
        if gpu_ids is not None or self._has_nvidia_runtime():
            default_config['device_requests'] = self._setup_gpu_allocation(gpu_ids)
        
        # Setup memory limit
        if memory_limit:
            default_config['mem_limit'] = memory_limit
        
        # Setup port mapping for API server
        if api_port is None:
            api_port = self._allocate_port(framework)
        
        default_config['ports'] = {
            f'{api_port}/tcp': api_port,
        }
        self._port_mapping[framework.value] = api_port
        
        # Merge with user config
        if config:
            default_config.update(config)
        
        self.logger.info(f"Creating container: {container_name}")
        self.logger.info(f"  Image: {image_name}")
        self.logger.info(f"  API Port: {api_port}")
        self.logger.info(f"  GPUs: {gpu_ids or 'all'}")
        
        try:
            container = self.client.containers.create(**default_config)
            self._containers[framework.value] = container.id
            self.logger.info(f"Created container: {container.short_id}")
            return container.id
            
        except APIError as e:
            self.logger.error(f"Failed to create container: {e}")
            raise

    def start_container(self, container_id: str) -> None:
        """
        Start a container.
        
        Args:
            container_id: Container ID to start
        
        Raises:
            NotFound: If container doesn't exist
        """
        try:
            container = self.client.containers.get(container_id)
            container.start()
            self.logger.info(f"Started container: {container.short_id}")
        except NotFound:
            self.logger.error(f"Container not found: {container_id}")
            raise
        except APIError as e:
            self.logger.error(f"Failed to start container: {e}")
            raise

    def stop_container(
        self,
        container_id: str,
        timeout: int = 10,
    ) -> None:
        """
        Stop a running container.
        
        Args:
            container_id: Container ID to stop
            timeout: Seconds to wait before killing container
        
        Raises:
            NotFound: If container doesn't exist
        """
        try:
            container = self.client.containers.get(container_id)
            container.stop(timeout=timeout)
            self.logger.info(f"Stopped container: {container.short_id}")
        except NotFound:
            self.logger.error(f"Container not found: {container_id}")
            raise
        except APIError as e:
            self.logger.error(f"Failed to stop container: {e}")
            raise

    def restart_container(
        self,
        container_id: str,
        timeout: int = 10,
    ) -> None:
        """
        Restart a container.
        
        Args:
            container_id: Container ID to restart
            timeout: Seconds to wait before killing container
        
        Raises:
            NotFound: If container doesn't exist
        """
        try:
            container = self.client.containers.get(container_id)
            container.restart(timeout=timeout)
            self.logger.info(f"Restarted container: {container.short_id}")
        except NotFound:
            self.logger.error(f"Container not found: {container_id}")
            raise
        except APIError as e:
            self.logger.error(f"Failed to restart container: {e}")
            raise

    def remove_container(
        self,
        container_id: str,
        force: bool = False,
        volumes: bool = False,
    ) -> None:
        """
        Remove a container.
        
        Args:
            container_id: Container ID to remove
            force: Force removal even if running
            volumes: Remove associated volumes
        
        Raises:
            NotFound: If container doesn't exist
        """
        try:
            container = self.client.containers.get(container_id)
            container.remove(force=force, v=volumes)
            
            # Remove from registry
            for fw, cid in list(self._containers.items()):
                if cid == container_id:
                    del self._containers[fw]
                    if fw in self._port_mapping:
                        del self._port_mapping[fw]
                    break
            
            self.logger.info(f"Removed container: {container.short_id}")
        except NotFound:
            self.logger.error(f"Container not found: {container_id}")
            raise
        except APIError as e:
            self.logger.error(f"Failed to remove container: {e}")
            raise

    def get_container_status(self, container_id: str) -> ContainerStatus:
        """
        Get container status.
        
        Args:
            container_id: Container ID to check
        
        Returns:
            Container status enum
        """
        try:
            container = self.client.containers.get(container_id)
            status = container.status.lower()
            return ContainerStatus(status)
        except NotFound:
            return ContainerStatus.NOT_EXIST
        except ValueError:
            self.logger.warning(f"Unknown container status: {status}")
            return ContainerStatus.NOT_EXIST

    def list_containers(
        self,
        all_containers: bool = True,
    ) -> List[Dict[str, Any]]:
        """
        List containers managed by this DockerManager.
        
        Args:
            all_containers: Include stopped containers
        
        Returns:
            List of container information dictionaries
        """
        result = []
        
        for framework_name, container_id in self._containers.items():
            try:
                container = self.client.containers.get(container_id)
                
                if not all_containers and container.status != 'running':
                    continue
                
                container_info = {
                    'id': container.short_id,
                    'name': container.name,
                    'framework': framework_name,
                    'status': container.status,
                    'image': container.image.tags[0] if container.image.tags else 'N/A',
                    'ports': self._port_mapping.get(framework_name),
                    'created': container.attrs['Created'],
                }
                result.append(container_info)
            except NotFound:
                self.logger.warning(f"Container {container_id} not found")
                continue
        
        return result

    # ============================================================================
    # Process Management
    # ============================================================================

    def exec_command(
        self,
        container_id: str,
        command: Union[str, List[str]],
        workdir: Optional[str] = None,
        environment: Optional[Dict[str, str]] = None,
        stream: bool = False,
    ) -> Union[str, Any]:
        """
        Execute command inside a container.
        
        Args:
            container_id: Container ID to execute command in
            command: Command to execute (string or list)
            workdir: Working directory for command
            environment: Environment variables
            stream: Stream output (returns generator if True)
        
        Returns:
            Command output (string or generator)
        
        Raises:
            NotFound: If container doesn't exist
        """
        try:
            container = self.client.containers.get(container_id)
            
            result = container.exec_run(
                cmd=command,
                workdir=workdir,
                environment=environment,
                stream=stream,
                demux=True,  # Separate stdout/stderr
            )
            
            if stream:
                return result.output
            else:
                exit_code, output = result
                if exit_code != 0:
                    self.logger.warning(
                        f"Command exited with code {exit_code}: {command}"
                    )
                return output.decode('utf-8') if output else ''
            
        except NotFound:
            self.logger.error(f"Container not found: {container_id}")
            raise
        except APIError as e:
            self.logger.error(f"Failed to execute command: {e}")
            raise

    def start_framework_process(
        self,
        container_id: str,
        framework: FrameworkType,
        config: Dict[str, Any],
    ) -> Dict[str, Any]:
        """
        Start a framework-specific process inside container.
        
        This method starts the main process for each framework (e.g., API server,
        inference server, etc.) based on framework type and configuration.
        
        Args:
            container_id: Container ID to start process in
            framework: Framework type
            config: Framework-specific configuration
        
        Returns:
            Process information dictionary
        
        Example:
            >>> config = {
            ...     'mode': 'inference',
            ...     'model_path': '/models/lerobot_policy',
            ...     'port': 8000,
            ... }
            >>> process_info = manager.start_framework_process(
            ...     container_id, FrameworkType.LEROBOT, config
            ... )
        """
        self.logger.info(f"Starting {framework.value} process in container {container_id}")
        
        # Framework-specific startup commands
        startup_commands = {
            FrameworkType.LEROBOT: self._get_lerobot_startup_cmd,
            FrameworkType.GROOT_N15: self._get_groot_startup_cmd,
            FrameworkType.PI0: self._get_pi0_startup_cmd,
        }
        
        if framework not in startup_commands:
            raise ValueError(f"Unsupported framework: {framework}")
        
        cmd = startup_commands[framework](config)
        
        # Execute startup command in background
        self.exec_command(
            container_id,
            f"nohup {cmd} > /tmp/{framework.value}.log 2>&1 &",
            workdir="/workspace",
        )
        
        # Wait for process to start
        time.sleep(2)
        
        # Verify process is running
        check_cmd = f"pgrep -f '{framework.value}'"
        output = self.exec_command(container_id, check_cmd)
        
        if output.strip():
            pid = output.strip().split('\n')[0]
            process_info = {
                'framework': framework.value,
                'pid': pid,
                'config': config,
                'log_file': f'/tmp/{framework.value}.log',
            }
            self.logger.info(f"Started {framework.value} process (PID: {pid})")
            return process_info
        else:
            raise RuntimeError(f"Failed to start {framework.value} process")

    def monitor_process(
        self,
        container_id: str,
        process_id: str,
    ) -> Dict[str, Any]:
        """
        Monitor a process inside container.
        
        Args:
            container_id: Container ID
            process_id: Process ID to monitor
        
        Returns:
            Process status information
        """
        try:
            # Check if process is running
            check_cmd = f"ps -p {process_id} -o pid,vsz,rss,pcpu,pmem,etime,comm"
            output = self.exec_command(container_id, check_cmd)
            
            if not output or 'PID' not in output:
                return {'status': 'not_running', 'pid': process_id}
            
            # Parse output
            lines = output.strip().split('\n')
            if len(lines) < 2:
                return {'status': 'not_running', 'pid': process_id}
            
            fields = lines[1].split()
            
            return {
                'status': 'running',
                'pid': fields[0],
                'memory_vsz': fields[1],
                'memory_rss': fields[2],
                'cpu_percent': fields[3],
                'mem_percent': fields[4],
                'elapsed_time': fields[5],
                'command': ' '.join(fields[6:]),
            }
            
        except Exception as e:
            self.logger.error(f"Failed to monitor process: {e}")
            return {'status': 'error', 'error': str(e)}

    # ============================================================================
    # Network & Communication
    # ============================================================================

    def connect_containers(
        self,
        container_ids: List[str],
        network: Optional[str] = None,
    ) -> None:
        """
        Connect multiple containers to the same network.
        
        Args:
            container_ids: List of container IDs to connect
            network: Network name (uses default if None)
        """
        network_name = network or self.network_name
        
        try:
            net = self.client.networks.get(network_name)
            
            for container_id in container_ids:
                try:
                    container = self.client.containers.get(container_id)
                    net.connect(container)
                    self.logger.info(
                        f"Connected container {container.short_id} to {network_name}"
                    )
                except APIError as e:
                    self.logger.warning(
                        f"Failed to connect container {container_id}: {e}"
                    )
                    
        except NotFound:
            self.logger.error(f"Network not found: {network_name}")
            raise

    def get_container_ip(self, container_id: str) -> Optional[str]:
        """
        Get container IP address in the network.
        
        Args:
            container_id: Container ID
        
        Returns:
            IP address or None if not found
        """
        try:
            container = self.client.containers.get(container_id)
            networks = container.attrs['NetworkSettings']['Networks']
            
            if self.network_name in networks:
                return networks[self.network_name]['IPAddress']
            
            return None
            
        except (NotFound, KeyError) as e:
            self.logger.error(f"Failed to get container IP: {e}")
            return None

    def setup_communication(
        self,
        source_framework: FrameworkType,
        target_framework: FrameworkType,
        protocol: CommunicationProtocol = CommunicationProtocol.ZMQ,
    ) -> Dict[str, Any]:
        """
        Setup communication between two framework containers.
        
        Args:
            source_framework: Source framework
            target_framework: Target framework
            protocol: Communication protocol to use (default: ZMQ)
        
        Returns:
            Communication configuration dictionary
        """
        source_id = self._containers.get(source_framework.value)
        target_id = self._containers.get(target_framework.value)
        
        if not source_id or not target_id:
            raise ValueError("Both containers must exist")
        
        # Get connection information
        target_ip = self.get_container_ip(target_id)
        target_port = self._port_mapping.get(target_framework.value)
        
        comm_config = {
            'protocol': protocol.value,
            'source': source_framework.value,
            'target': target_framework.value,
            'target_ip': target_ip,
            'target_port': target_port,
        }
        
        if protocol == CommunicationProtocol.ZMQ:
            # ZMQ is the primary communication protocol
            comm_config['endpoint'] = f"tcp://{target_ip}:{target_port}"
            comm_config['pattern'] = 'REQ-REP'  # Request-Reply pattern
        elif protocol == CommunicationProtocol.SHARED_VOLUME:
            # For large data transfer (datasets, models)
            shared_path = self.shared_volume_path / "shared_data"
            comm_config['shared_path'] = str(shared_path)
        
        self.logger.info(
            f"Setup communication: {source_framework.value} -> "
            f"{target_framework.value} ({protocol.value})"
        )
        
        return comm_config

    # ============================================================================
    # Resource Management
    # ============================================================================

    def allocate_gpu(
        self,
        container_id: str,
        gpu_ids: List[int],
    ) -> None:
        """
        Allocate specific GPUs to a container.
        
        Note: This requires restarting the container.
        
        Args:
            container_id: Container ID
            gpu_ids: List of GPU IDs to allocate
        """
        self.logger.warning(
            "GPU allocation requires container restart. "
            "This is not recommended for running containers."
        )
        # GPU allocation must be set during container creation
        # This is a limitation of Docker
        raise NotImplementedError(
            "Dynamic GPU allocation not supported. "
            "Recreate container with desired GPU configuration."
        )

    def set_memory_limits(
        self,
        container_id: str,
        memory_limit: str,
        memory_swap: Optional[str] = None,
    ) -> None:
        """
        Update memory limits for a container.
        
        Args:
            container_id: Container ID
            memory_limit: Memory limit (e.g., '4g', '512m')
            memory_swap: Memory + swap limit
        """
        try:
            container = self.client.containers.get(container_id)
            container.update(
                mem_limit=memory_limit,
                memswap_limit=memory_swap or memory_limit,
            )
            self.logger.info(
                f"Updated memory limit for {container.short_id}: {memory_limit}"
            )
        except NotFound:
            self.logger.error(f"Container not found: {container_id}")
            raise
        except APIError as e:
            self.logger.error(f"Failed to update memory limits: {e}")
            raise

    def monitor_resources(
        self,
        container_id: str,
    ) -> Dict[str, Any]:
        """
        Monitor resource usage of a container.
        
        Args:
            container_id: Container ID
        
        Returns:
            Resource usage statistics
        """
        try:
            container = self.client.containers.get(container_id)
            stats = container.stats(stream=False)
            
            # Parse CPU usage
            cpu_delta = (
                stats['cpu_stats']['cpu_usage']['total_usage'] -
                stats['precpu_stats']['cpu_usage']['total_usage']
            )
            system_delta = (
                stats['cpu_stats']['system_cpu_usage'] -
                stats['precpu_stats']['system_cpu_usage']
            )
            cpu_count = stats['cpu_stats'].get('online_cpus', 1)
            
            cpu_percent = 0.0
            if system_delta > 0:
                cpu_percent = (cpu_delta / system_delta) * cpu_count * 100.0
            
            # Parse memory usage
            memory_usage = stats['memory_stats']['usage']
            memory_limit = stats['memory_stats']['limit']
            memory_percent = (memory_usage / memory_limit) * 100.0
            
            # Parse network I/O
            networks = stats.get('networks', {})
            net_rx = sum(net['rx_bytes'] for net in networks.values())
            net_tx = sum(net['tx_bytes'] for net in networks.values())
            
            # Parse block I/O
            block_io = stats.get('blkio_stats', {}).get('io_service_bytes_recursive', [])
            disk_read = sum(
                item['value'] for item in block_io if item.get('op') == 'Read'
            )
            disk_write = sum(
                item['value'] for item in block_io if item.get('op') == 'Write'
            )
            
            return {
                'cpu_percent': round(cpu_percent, 2),
                'memory_usage': memory_usage,
                'memory_limit': memory_limit,
                'memory_percent': round(memory_percent, 2),
                'network_rx_bytes': net_rx,
                'network_tx_bytes': net_tx,
                'disk_read_bytes': disk_read,
                'disk_write_bytes': disk_write,
            }
            
        except NotFound:
            self.logger.error(f"Container not found: {container_id}")
            raise
        except (KeyError, APIError) as e:
            self.logger.error(f"Failed to monitor resources: {e}")
            return {}

    # ============================================================================
    # Helper Methods
    # ============================================================================

    def _setup_volumes(self, framework: FrameworkType) -> List[Mount]:
        """Setup volume mounts for a framework container."""
        volumes = []
        
        # Shared data volume (for datasets, models, etc.)
        shared_data_path = self.shared_volume_path / "shared_data"
        shared_data_path.mkdir(exist_ok=True)
        
        volumes.append(
            Mount(
                target="/workspace/shared_data",
                source=str(shared_data_path),
                type="bind",
            )
        )
        
        # Framework-specific volume
        framework_path = self.shared_volume_path / framework.value
        framework_path.mkdir(exist_ok=True)
        
        volumes.append(
            Mount(
                target=f"/workspace/{framework.value}_data",
                source=str(framework_path),
                type="bind",
            )
        )
        
        # Hugging Face cache (for LeRobot)
        if framework == FrameworkType.LEROBOT:
            hf_cache_path = Path.home() / ".cache/huggingface"
            hf_cache_path.mkdir(parents=True, exist_ok=True)
            
            volumes.append(
                Mount(
                    target="/root/.cache/huggingface",
                    source=str(hf_cache_path),
                    type="bind",
                )
            )
        
        return volumes

    def _setup_gpu_allocation(
        self,
        gpu_ids: Optional[List[int]] = None,
    ) -> List[DeviceRequest]:
        """Setup GPU device requests for container."""
        if gpu_ids:
            # Specific GPUs
            device_ids = [str(gpu_id) for gpu_id in gpu_ids]
            return [
                DeviceRequest(
                    device_ids=device_ids,
                    capabilities=[['gpu']],
                )
            ]
        else:
            # All GPUs
            return [
                DeviceRequest(
                    count=-1,  # All GPUs
                    capabilities=[['gpu']],
                )
            ]

    def _has_nvidia_runtime(self) -> bool:
        """Check if NVIDIA container runtime is available."""
        try:
            info = self.client.info()
            runtimes = info.get('Runtimes', {})
            return 'nvidia' in runtimes
        except Exception:
            return False

    def _allocate_port(self, framework: FrameworkType) -> int:
        """Allocate an available port for framework ZMQ server."""
        # Use predefined ports for each framework
        if framework in self.FRAMEWORK_PORTS:
            return self.FRAMEWORK_PORTS[framework]
        
        # Fallback: use base port + offset
        base_port = self.DEFAULT_API_PORT_BASE
        offset = len(self._port_mapping)
        return base_port + offset

    def _get_lerobot_startup_cmd(self, config: Dict[str, Any]) -> str:
        """Get LeRobot startup command with ZMQ server."""
        mode = config.get('mode', 'inference')
        port = config.get('port', 5555)  # ZMQ port
        
        if mode == 'inference':
            model_path = config.get('model_path', '/models/lerobot_policy')
            return (
                f"python -m lerobot.scripts.zmq_inference_server "
                f"--model_path {model_path} --port {port}"
            )
        elif mode == 'training':
            return f"python -m lerobot.scripts.zmq_train_server --port {port}"
        else:
            raise ValueError(f"Unknown mode: {mode}")

    def _get_groot_startup_cmd(self, config: Dict[str, Any]) -> str:
        """Get GR00T N1.5 startup command with ZMQ server."""
        mode = config.get('mode', 'inference')
        port = config.get('port', 5556)  # ZMQ port
        model_path = config.get('model_path', 'nvidia/GR00T-N1.5-3B')
        embodiment_tag = config.get('embodiment_tag', 'default')
        device = config.get('device', 'cuda')
        
        return (
            f"python scripts/zmq_inference_server.py "
            f"--port {port} "
            f"--model_path {model_path} "
            f"--embodiment_tag {embodiment_tag} "
            f"--device {device}"
        )

    def _get_pi0_startup_cmd(self, config: Dict[str, Any]) -> str:
        """Get Pi0 startup command with ZMQ server."""
        mode = config.get('mode', 'inference')
        port = config.get('port', 5557)  # ZMQ port
        return f"python -m pi0.zmq_server --mode {mode} --port {port}"

    # ============================================================================
    # Context Manager Support
    # ============================================================================

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - cleanup resources."""
        self.cleanup()

    def cleanup(self):
        """Cleanup all managed containers and networks."""
        self.logger.info("Cleaning up Docker resources...")
        
        # Stop and remove all managed containers
        for framework_name, container_id in list(self._containers.items()):
            try:
                self.stop_container(container_id, timeout=5)
                self.remove_container(container_id, force=True)
            except Exception as e:
                self.logger.warning(
                    f"Failed to cleanup container {framework_name}: {e}"
                )
        
        # Remove network if it exists
        if self._network:
            try:
                self._network.remove()
                self.logger.info(f"Removed network: {self.network_name}")
            except Exception as e:
                self.logger.warning(f"Failed to remove network: {e}")

    def __del__(self):
        """Destructor - ensure cleanup."""
        if hasattr(self, 'client'):
            self.client.close()
