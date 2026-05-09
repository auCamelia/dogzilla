import os
import threading
import webbrowser
import http.server
import socketserver

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class WebServer(Node):
    def __init__(self):
        super().__init__('dogzilla_web_server')
        self.declare_parameter('port', 8080)
        self.declare_parameter('open_browser', True)

        port = self.get_parameter('port').value
        open_browser = self.get_parameter('open_browser').value

        web_dir = os.path.join(
            get_package_share_directory('dogzilla_teleop'), 'web')

        handler = _make_handler(web_dir)
        self._httpd = socketserver.TCPServer(('', port), handler)
        self._httpd.allow_reuse_address = True

        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'PC      → http://localhost:{port}/teleop.html')
        self.get_logger().info(f'Mobile  → http://<pi-ip>:{port}/teleop.html  (PWA)')
        self.get_logger().info(f'Watch   → http://<pi-ip>:{port}/watch.html   (Samsung Internet)')

        if open_browser:
            webbrowser.open(f'http://localhost:{port}/teleop.html')

    def destroy_node(self):
        self._httpd.shutdown()
        super().destroy_node()


def _make_handler(directory):
    class Handler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=directory, **kwargs)

        def log_message(self, *_):
            pass  # silence access logs

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = WebServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
