#!/usr/bin/env python3

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Motor Control</title>
<style>
  body { font-family: -apple-system, sans-serif; text-align: center; padding-top: 12vh; background: #111; color: #eee; }
  button { font-size: 1.5rem; padding: 1.2rem 2.4rem; margin: 0.6rem; border: none; border-radius: 14px; min-width: 40vw; }
  #on  { background: #1b7a3d; color: white; }
  #off { background: #8a1f1f; color: white; }
  button:active { filter: brightness(0.8); }
  #status { margin-top: 2rem; font-size: 1.1rem; color: #aaa; }
  #dot { display: inline-block; width: 0.7em; height: 0.7em; border-radius: 50%; margin-right: 0.4em; background: #666; }
</style>
</head>
<body>
  <h2>M3508 Motor</h2>
  <div>
    <button id="on" onclick="cmd('on')">START</button>
    <button id="off" onclick="cmd('off')">STOP</button>
  </div>
  <div id="status"><span id="dot"></span><span id="text">loading...</span></div>
<script>
async function cmd(which) {
  document.getElementById('text').textContent = 'sending...';
  try { await fetch('/' + which, {method: 'POST'}); } catch (e) {}
  refresh();
}
async function refresh() {
  try {
    const r = await fetch('/state');
    const s = await r.json();
    const dot = document.getElementById('dot');
    const text = document.getElementById('text');
    if (!s.connected) {
      dot.style.background = '#666';
      text.textContent = 'motor not responding';
    } else {
      dot.style.background = s.enabled ? '#2ecc71' : '#e74c3c';
      text.textContent = (s.enabled ? 'RUNNING' : 'STOPPED') +
        '  |  ' + s.velocity_hz.toFixed(2) + ' Hz  |  updated ' + s.age.toFixed(1) + 's ago';
    }
  } catch (e) {
    document.getElementById('text').textContent = 'lost connection to server';
  }
}
refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>"""


class WebControlNode(Node):

    def __init__(self):
        super().__init__("web_control_node")
        self.declare_parameter("port", 1234)
        self.declare_parameter("trigger_topic", "/motor/trigger")
        self.declare_parameter("state_topic", "/motor/state")
        self.declare_parameter("bind", "127.0.0.1")   # 只监听本机; Cloudflare Tunnel 从本机转发进来
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.enabled = False
        self.connected = False
        self.velocity_hz = 0.0
        self.last_state_time = 0.0
        self.lock = threading.Lock()

        self.pub = self.create_publisher(Bool, p("trigger_topic"), 10)
        self.create_subscription(JointState, p("state_topic"), self._on_state, 10)

        self.port = int(p("port"))
        self.bind = p("bind")
        self.httpd = ThreadingHTTPServer((self.bind, self.port), self._make_handler())
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, name="web-control-http", daemon=True)
        self.http_thread.start()
        self.get_logger().info(f"web UI on http://{self.bind}:{self.port}  (trigger={p('trigger_topic')} state={p('state_topic')})")

    def _on_state(self, msg: JointState):
        with self.lock:
            self.connected = True
            self.last_state_time = time.monotonic()
            if msg.velocity:
                self.velocity_hz = msg.velocity[0] / (2.0 * 3.141592653589793)

    def _publish_trigger(self, value: bool):
        with self.lock:
            self.enabled = value
        self.pub.publish(Bool(data=value))
        self.get_logger().info(f"web UI -> trigger={int(value)}")

    def _snapshot(self):
        with self.lock:
            age = time.monotonic() - self.last_state_time if self.connected else float("inf")
            connected = self.connected and age < 3.0   # 3 秒没更新就认为掉线
            return {
                "enabled": self.enabled,
                "connected": connected,
                "velocity_hz": self.velocity_hz if connected else 0.0,
                "age": 0.0 if age == float("inf") else age,
            }

    def _make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                node.get_logger().debug("%s - %s" % (self.address_string(), fmt % args))

            def _send(self, code, body, content_type="text/plain"):
                data = body.encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if self.path == "/":
                    self._send(200, PAGE, "text/html; charset=utf-8")
                elif self.path == "/state":
                    self._send(200, json.dumps(node._snapshot()), "application/json")
                else:
                    self._send(404, "not found")

            def do_POST(self):
                if self.path == "/on":
                    node._publish_trigger(True)
                    self._send(200, "ok")
                elif self.path == "/off":
                    node._publish_trigger(False)
                    self._send(200, "ok")
                else:
                    self._send(404, "not found")

        return Handler

    def destroy_node(self):
        try:
            self.httpd.shutdown()
            self.http_thread.join(timeout=2.0)
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
