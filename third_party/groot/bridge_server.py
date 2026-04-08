#!/usr/bin/env python3
"""TCP Bridge Server for inference service calls.

Runs inside the executor process as a background thread.
Receives JSON requests over TCP and calls the inference handlers DIRECTLY,
completely bypassing Zenoh to avoid stale queryable routing issues.

Protocol:
    Request:  4-byte big-endian length + JSON payload
    Response: 4-byte big-endian length + JSON payload

Supported actions:
    start_inference:   {"action": "start_inference", "model_path": ..., ...}
    get_action_chunk:  {"action": "get_action_chunk", "task_instruction": ...}
    stop:              {"action": "stop"}
"""
import json
import logging
import socket
import struct
import threading
import types

logger = logging.getLogger("bridge_server")


def _recv_exact(sock, n):
    """Receive exactly n bytes."""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def _handle_client(conn, addr, inference, stop_fn):
    """Handle a single TCP client connection (persistent)."""
    try:
        while True:
            # Read 4-byte length header
            header = _recv_exact(conn, 4)
            if not header:
                break
            msg_len = struct.unpack(">I", header)[0]
            if msg_len > 10_000_000:  # 10MB safety limit
                break

            # Read JSON payload
            payload = _recv_exact(conn, msg_len)
            if not payload:
                break

            try:
                request = json.loads(payload)
                action = request.get("action", "")
                result = _dispatch(action, request, inference, stop_fn)
            except Exception as e:
                logger.error(f"Bridge request error: {e}", exc_info=True)
                result = {"success": False, "message": str(e)}

            # Send response
            resp_bytes = json.dumps(result).encode("utf-8")
            conn.sendall(struct.pack(">I", len(resp_bytes)) + resp_bytes)
    except (ConnectionResetError, BrokenPipeError):
        pass
    except Exception as e:
        logger.error(f"Bridge client error: {e}")
    finally:
        conn.close()


def _dispatch(action, data, inference, stop_fn):
    """Route action to the correct inference handler (no Zenoh)."""
    if action == "start_inference":
        # Build a simple namespace object to mimic the ROS2 request
        req = types.SimpleNamespace(
            model_path=data.get("model_path", ""),
            embodiment_tag=data.get("embodiment_tag", ""),
            robot_type=data.get("robot_type", ""),
            task_instruction=data.get("task_instruction", ""),
        )
        result = inference.load_policy(req)
        if isinstance(result, dict):
            return result
        # If result is a response object
        return {
            "success": bool(getattr(result, "success", False)),
            "message": str(getattr(result, "message", "")),
            "action_keys": list(getattr(result, "action_keys", [])),
        }

    elif action == "get_action_chunk":
        req = types.SimpleNamespace(
            task_instruction=data.get("task_instruction", ""),
        )
        result = inference.get_action_chunk(req)
        if isinstance(result, dict):
            return result
        return {
            "success": bool(getattr(result, "success", False)),
            "message": str(getattr(result, "message", "")),
            "action_chunk": list(getattr(result, "action_chunk", [])),
            "chunk_size": int(getattr(result, "chunk_size", 0)),
            "action_dim": int(getattr(result, "action_dim", 0)),
        }

    elif action == "stop":
        try:
            stop_fn()
            return {"success": True, "message": "Stopped"}
        except Exception as e:
            return {"success": False, "message": str(e)}

    else:
        return {"success": False, "message": f"Unknown action: {action}"}


def start_bridge_server(port, inference, stop_fn):
    """Start the TCP bridge server as a background daemon thread.

    Args:
        port: TCP port to listen on.
        inference: GR00TInference instance to call directly.
        stop_fn: Callable to invoke for stop action.
    """
    def _serve():
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", port))
        srv.listen(4)
        logger.info(f"Bridge server listening on port {port}")

        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            t = threading.Thread(
                target=_handle_client,
                args=(conn, addr, inference, stop_fn),
                daemon=True,
            )
            t.start()

    thread = threading.Thread(target=_serve, daemon=True)
    thread.start()
    return thread
