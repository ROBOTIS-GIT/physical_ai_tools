"""
ZMQ Communication Helper for Physical AI Tools

This module provides ZMQ-based communication utilities for Physical AI Server
to communicate with framework containers (LeRobot, GR00T, Pi0, etc.).

ZMQ Pattern: REQ-REP (Request-Reply)
- Physical AI Server: ZMQ Client (REQ)
- Framework Containers: ZMQ Server (REP)

Message Format (JSON):
{
    "type": "inference" | "train" | "command",
    "data": {...},
    "timestamp": <unix_timestamp>,
    "request_id": <uuid>
}
"""

import json
import logging
import time
import uuid
from enum import Enum
from typing import Any, Dict, Optional

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False
    logging.warning("ZMQ not available. Install with: pip install pyzmq")


class MessageType(Enum):
    """ZMQ message types"""
    INFERENCE = "inference"
    TRAIN = "train"
    COMMAND = "command"
    STATUS = "status"
    HEALTH = "health"


class ZMQClient:
    """
    ZMQ Client for Physical AI Server to communicate with framework containers.
    
    This client uses REQ-REP pattern for synchronous request-reply communication.
    Each framework container runs a ZMQ server that this client connects to.
    
    Attributes:
        endpoint: ZMQ endpoint (e.g., 'tcp://172.17.0.2:5555')
        context: ZMQ context
        socket: ZMQ REQ socket
        timeout: Request timeout in milliseconds
    """
    
    DEFAULT_TIMEOUT = 5000  # 5 seconds
    
    def __init__(
        self,
        endpoint: str,
        timeout: int = DEFAULT_TIMEOUT,
        logger: Optional[logging.Logger] = None,
    ):
        """
        Initialize ZMQ client.
        
        Args:
            endpoint: ZMQ endpoint (e.g., 'tcp://172.17.0.2:5555')
            timeout: Request timeout in milliseconds
            logger: Logger instance
        
        Raises:
            ImportError: If pyzmq is not installed
        """
        if not ZMQ_AVAILABLE:
            raise ImportError("pyzmq not installed. Install with: pip install pyzmq")
        
        self.endpoint = endpoint
        self.timeout = timeout
        self.logger = logger or logging.getLogger(__name__)
        
        # Create ZMQ context and socket
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        
        # Set socket options
        self.socket.setsockopt(zmq.RCVTIMEO, timeout)
        self.socket.setsockopt(zmq.SNDTIMEO, timeout)
        self.socket.setsockopt(zmq.LINGER, 0)
        
        # Connect to server
        self.socket.connect(endpoint)
        self.logger.info(f"ZMQ client connected to {endpoint}")
    
    def send_request(
        self,
        message_type: MessageType,
        data: Dict[str, Any],
        timeout: Optional[int] = None,
    ) -> Optional[Dict[str, Any]]:
        """
        Send request to framework container and wait for response.
        
        Args:
            message_type: Type of message
            data: Message data
            timeout: Custom timeout in milliseconds (uses default if None)
        
        Returns:
            Response dictionary or None on timeout/error
        
        Example:
            >>> client = ZMQClient('tcp://172.17.0.2:5555')
            >>> response = client.send_request(
            ...     MessageType.INFERENCE,
            ...     {'observation': [...], 'action': [...]}
            ... )
        """
        # Create request message
        request = {
            'type': message_type.value,
            'data': data,
            'timestamp': time.time(),
            'request_id': str(uuid.uuid4()),
        }
        
        # Set custom timeout if provided
        if timeout is not None:
            old_timeout = self.socket.getsockopt(zmq.RCVTIMEO)
            self.socket.setsockopt(zmq.RCVTIMEO, timeout)
        
        try:
            # Send request
            self.socket.send_json(request)
            self.logger.debug(
                f"Sent {message_type.value} request: {request['request_id']}"
            )
            
            # Wait for response
            response = self.socket.recv_json()
            self.logger.debug(
                f"Received response: {response.get('request_id', 'unknown')}"
            )
            
            return response
            
        except zmq.Again:
            self.logger.error(f"Request timeout after {timeout or self.timeout}ms")
            # Reset socket on timeout
            self._reset_socket()
            return None
            
        except zmq.ZMQError as e:
            self.logger.error(f"ZMQ error: {e}")
            self._reset_socket()
            return None
            
        except Exception as e:
            self.logger.error(f"Unexpected error: {e}")
            return None
            
        finally:
            # Restore timeout
            if timeout is not None:
                self.socket.setsockopt(zmq.RCVTIMEO, old_timeout)
    
    def inference(
        self,
        observation: Dict[str, Any],
        **kwargs,
    ) -> Optional[Dict[str, Any]]:
        """
        Send inference request.
        
        Args:
            observation: Observation data (images, robot state, etc.)
            **kwargs: Additional inference parameters
        
        Returns:
            Inference result (actions, etc.) or None on error
        """
        data = {
            'observation': observation,
            **kwargs,
        }
        return self.send_request(MessageType.INFERENCE, data)
    
    def train(
        self,
        config: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        """
        Send training request.
        
        Args:
            config: Training configuration
        
        Returns:
            Training status or None on error
        """
        return self.send_request(MessageType.TRAIN, config)
    
    def command(
        self,
        command: str,
        params: Optional[Dict[str, Any]] = None,
    ) -> Optional[Dict[str, Any]]:
        """
        Send custom command.
        
        Args:
            command: Command name
            params: Command parameters
        
        Returns:
            Command result or None on error
        """
        data = {
            'command': command,
            'params': params or {},
        }
        return self.send_request(MessageType.COMMAND, data)
    
    def health_check(self) -> bool:
        """
        Check if framework container is healthy.
        
        Returns:
            True if healthy, False otherwise
        """
        try:
            response = self.send_request(
                MessageType.HEALTH,
                {},
                timeout=1000,  # 1 second timeout for health check
            )
            return response is not None and response.get('status') == 'ok'
        except Exception:
            return False
    
    def _reset_socket(self):
        """Reset socket after error or timeout."""
        try:
            self.socket.close()
            self.socket = self.context.socket(zmq.REQ)
            self.socket.setsockopt(zmq.RCVTIMEO, self.timeout)
            self.socket.setsockopt(zmq.SNDTIMEO, self.timeout)
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.connect(self.endpoint)
            self.logger.debug("Socket reset and reconnected")
        except Exception as e:
            self.logger.error(f"Failed to reset socket: {e}")
    
    def close(self):
        """Close ZMQ socket and context."""
        try:
            self.socket.close()
            self.context.term()
            self.logger.info("ZMQ client closed")
        except Exception as e:
            self.logger.error(f"Error closing ZMQ client: {e}")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
    
    def __del__(self):
        """Destructor."""
        self.close()


class ZMQServer:
    """
    ZMQ Server for framework containers.
    
    This server runs inside each framework container and handles requests
    from Physical AI Server. Implements REP (reply) side of REQ-REP pattern.
    
    Example:
        >>> def handle_inference(data):
        ...     # Run inference
        ...     result = model.predict(data['observation'])
        ...     return {'action': result}
        >>> 
        >>> server = ZMQServer('tcp://*:5555')
        >>> server.register_handler(MessageType.INFERENCE, handle_inference)
        >>> server.run()
    """
    
    def __init__(
        self,
        endpoint: str = 'tcp://*:5555',
        logger: Optional[logging.Logger] = None,
    ):
        """
        Initialize ZMQ server.
        
        Args:
            endpoint: ZMQ endpoint to bind (e.g., 'tcp://*:5555')
            logger: Logger instance
        """
        if not ZMQ_AVAILABLE:
            raise ImportError("pyzmq not installed. Install with: pip install pyzmq")
        
        self.endpoint = endpoint
        self.logger = logger or logging.getLogger(__name__)
        
        # Create ZMQ context and socket
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        
        # Bind to endpoint
        self.socket.bind(endpoint)
        self.logger.info(f"ZMQ server listening on {endpoint}")
        
        # Handler registry
        self.handlers = {}
        
        # Register default health check handler
        self.register_handler(MessageType.HEALTH, self._health_check_handler)
        
        # Running flag
        self.running = False
    
    def register_handler(
        self,
        message_type: MessageType,
        handler: callable,
    ):
        """
        Register message handler.
        
        Args:
            message_type: Message type to handle
            handler: Handler function that takes data dict and returns result dict
        """
        self.handlers[message_type.value] = handler
        self.logger.info(f"Registered handler for {message_type.value}")
    
    def _health_check_handler(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Default health check handler."""
        return {'status': 'ok', 'timestamp': time.time()}
    
    def run(self):
        """
        Run server loop to handle incoming requests.
        
        This method blocks and processes requests until stopped.
        """
        self.running = True
        self.logger.info("ZMQ server started")
        
        try:
            while self.running:
                try:
                    # Wait for request
                    request = self.socket.recv_json()
                    
                    message_type = request.get('type')
                    data = request.get('data', {})
                    request_id = request.get('request_id', 'unknown')
                    
                    self.logger.debug(f"Received {message_type} request: {request_id}")
                    
                    # Handle request
                    if message_type in self.handlers:
                        try:
                            result = self.handlers[message_type](data)
                            response = {
                                'status': 'success',
                                'result': result,
                                'request_id': request_id,
                                'timestamp': time.time(),
                            }
                        except Exception as e:
                            self.logger.error(f"Handler error: {e}")
                            response = {
                                'status': 'error',
                                'error': str(e),
                                'request_id': request_id,
                                'timestamp': time.time(),
                            }
                    else:
                        response = {
                            'status': 'error',
                            'error': f'Unknown message type: {message_type}',
                            'request_id': request_id,
                            'timestamp': time.time(),
                        }
                    
                    # Send response
                    self.socket.send_json(response)
                    self.logger.debug(f"Sent response: {request_id}")
                    
                except zmq.ZMQError as e:
                    if e.errno == zmq.ETERM:
                        # Context terminated
                        break
                    self.logger.error(f"ZMQ error: {e}")
                    
                except KeyboardInterrupt:
                    self.logger.info("Received interrupt signal")
                    break
                    
                except Exception as e:
                    self.logger.error(f"Unexpected error: {e}")
                    
        finally:
            self.stop()
    
    def stop(self):
        """Stop server."""
        self.running = False
        try:
            self.socket.close()
            self.context.term()
            self.logger.info("ZMQ server stopped")
        except Exception as e:
            self.logger.error(f"Error stopping server: {e}")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()


class ZMQClientPool:
    """
    Pool of ZMQ clients for managing multiple framework connections.
    
    Physical AI Server uses this to maintain connections to all active
    framework containers.
    
    Example:
        >>> pool = ZMQClientPool()
        >>> pool.add_client('lerobot', 'tcp://172.17.0.2:5555')
        >>> pool.add_client('groot', 'tcp://172.17.0.3:5556')
        >>> 
        >>> # Send inference request to lerobot
        >>> result = pool.inference('lerobot', observation={'image': [...]})
    """
    
    def __init__(self, logger: Optional[logging.Logger] = None):
        """Initialize client pool."""
        self.clients: Dict[str, ZMQClient] = {}
        self.logger = logger or logging.getLogger(__name__)
    
    def add_client(
        self,
        name: str,
        endpoint: str,
        timeout: int = ZMQClient.DEFAULT_TIMEOUT,
    ):
        """
        Add a new client to the pool.
        
        Args:
            name: Client name (e.g., 'lerobot', 'groot')
            endpoint: ZMQ endpoint
            timeout: Request timeout
        """
        if name in self.clients:
            self.logger.warning(f"Client {name} already exists, replacing")
            self.clients[name].close()
        
        self.clients[name] = ZMQClient(endpoint, timeout, self.logger)
        self.logger.info(f"Added client {name}: {endpoint}")
    
    def remove_client(self, name: str):
        """Remove client from pool."""
        if name in self.clients:
            self.clients[name].close()
            del self.clients[name]
            self.logger.info(f"Removed client {name}")
    
    def get_client(self, name: str) -> Optional[ZMQClient]:
        """Get client by name."""
        return self.clients.get(name)
    
    def inference(
        self,
        name: str,
        observation: Dict[str, Any],
        **kwargs,
    ) -> Optional[Dict[str, Any]]:
        """Send inference request to specific framework."""
        client = self.get_client(name)
        if client:
            return client.inference(observation, **kwargs)
        else:
            self.logger.error(f"Client {name} not found")
            return None
    
    def health_check_all(self) -> Dict[str, bool]:
        """Check health of all clients."""
        results = {}
        for name, client in self.clients.items():
            results[name] = client.health_check()
        return results
    
    def close_all(self):
        """Close all clients."""
        for name, client in list(self.clients.items()):
            client.close()
        self.clients.clear()
        self.logger.info("Closed all clients")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close_all()
    
    def __del__(self):
        """Destructor."""
        self.close_all()
