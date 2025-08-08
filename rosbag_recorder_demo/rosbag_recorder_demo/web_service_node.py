import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
import json

import rclpy
from rclpy.node import Node

from rosbag_recorder_msgs.srv import StartRecording, StopRecording


class WebServiceNode(Node):
    def __init__(self):
        super().__init__('rosbag_recorder_web')
        self._start_cli = self.create_client(StartRecording, 'start_recording')
        self._stop_cli = self.create_client(StopRecording, 'stop_recording')

        self._server_thread = None
        self._server = None

    def start_http(self, host='0.0.0.0', port=8000):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def _send_json(self, code=200, payload=None):
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                if payload is not None:
                    self.wfile.write(json.dumps(payload).encode('utf-8'))

            def _send_html(self, code=200, html: str = ""):
                self.send_response(code)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.end_headers()
                self.wfile.write(html.encode('utf-8'))

            def do_GET(self):
                if self.path == '/' or self.path.startswith('/index.html'):
                    html = """
<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>ROSBag Recorder Demo</title>
  <style>
    body{font-family:system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial, sans-serif; margin:2rem;}
    h1{font-size:1.25rem}
    label{display:block;margin:.5rem 0 .25rem}
    input, textarea{width:100%;max-width:680px;padding:.5rem;border:1px solid #ccc;border-radius:6px}
    textarea{height:5rem}
    button{margin-top:1rem;padding:.5rem 1rem;border:0;border-radius:6px;background:#2563eb;color:#fff;cursor:pointer}
    button.secondary{background:#6b7280}
    .row{display:flex;gap:2rem;flex-wrap:wrap}
    .card{border:1px solid #e5e7eb;border-radius:8px;padding:1rem;max-width:720px}
    #status{margin-top:1rem;white-space:pre-wrap}
    footer{margin-top:2rem;color:#6b7280;font-size:.9rem}
  </style>
  <script>
    async function startRecording(){
      const uri = document.getElementById('uri').value.trim();
      const topicsText = document.getElementById('topics').value.trim();
      const topics = topicsText ? topicsText.split(/\\n|,/) .map(s=>s.trim()).filter(Boolean) : [];
      setStatus('Starting...');
      try{
        const res = await fetch('/start', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({uri, topics})});
        const json = await res.json();
        setStatus(JSON.stringify(json, null, 2));
      }catch(err){ setStatus(String(err)); }
    }
    async function stopRecording(){
      setStatus('Stopping...');
      try{
        const res = await fetch('/stop', {method:'POST'});
        const json = await res.json();
        setStatus(JSON.stringify(json, null, 2));
      }catch(err){ setStatus(String(err)); }
    }
    function setStatus(text){ document.getElementById('status').textContent = text; }
  </script>
  </head>
  <body>
    <h1>ROSBag Recorder Web UI</h1>
    <div class=\"row\">
      <div class=\"card\">
        <label for=\"uri\">Bag directory (uri):</label>
        <input id=\"uri\" placeholder=\"/tmp/my_bag\" />
        <label for=\"topics\">Topics (comma or newline separated):</label>
        <textarea id=\"topics\" placeholder=\"/joint_states\n/tf\"></textarea>
        <div>
          <button onclick=\"startRecording()\">Start Recording</button>
          <button class=\"secondary\" onclick=\"stopRecording()\">Stop Recording</button>
        </div>
        <pre id=\"status\"></pre>
      </div>
    </div>
    <footer>Point this UI at a running node providing services <code>/start_recording</code> and <code>/stop_recording</code>.</footer>
  </body>
  </html>
                    """
                    return self._send_html(200, html)
                if self.path.startswith('/health'):
                    return self._send_json(200, {'ok': True})
                return self._send_json(404, {'error': 'not found'})

            def do_POST(self):
                length = int(self.headers.get('content-length', '0') or '0')
                body = self.rfile.read(length).decode('utf-8') if length > 0 else ''
                try:
                    data = json.loads(body) if body else {}
                except Exception:
                    return self._send_json(400, {'error': 'invalid json'})

                if self.path == '/start':
                    uri = data.get('uri')
                    topics = data.get('topics', [])
                    if not isinstance(topics, list):
                        return self._send_json(400, {'error': 'topics must be a list of strings'})
                    if not uri:
                        return self._send_json(400, {'error': 'uri is required'})

                    if not node._start_cli.wait_for_service(timeout_sec=2.0):
                        return self._send_json(503, {'error': 'start_recording service unavailable'})
                    req = StartRecording.Request()
                    req.uri = uri
                    req.topics = topics
                    future = node._start_cli.call_async(req)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
                    if not future.done():
                        return self._send_json(504, {'error': 'start timeout'})
                    resp = future.result()
                    return self._send_json(200, {'success': resp.success, 'message': resp.message})

                if self.path == '/stop':
                    if not node._stop_cli.wait_for_service(timeout_sec=2.0):
                        return self._send_json(503, {'error': 'stop_recording service unavailable'})
                    req = StopRecording.Request()
                    future = node._stop_cli.call_async(req)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
                    if not future.done():
                        return self._send_json(504, {'error': 'stop timeout'})
                    resp = future.result()
                    return self._send_json(200, {'success': resp.success, 'message': resp.message})

                return self._send_json(404, {'error': 'not found'})

        self._server = HTTPServer((host, port), Handler)
        self.get_logger().info(f'HTTP server listening on {host}:{port}')

        def serve():
            try:
                self._server.serve_forever()
            except Exception as e:
                self.get_logger().error(f'HTTP server error: {e}')

        self._server_thread = threading.Thread(target=serve, daemon=True)
        self._server_thread.start()


def main():
    rclpy.init()
    node = WebServiceNode()
    node.start_http()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


