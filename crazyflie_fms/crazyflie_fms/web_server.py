"""UI 정적 파일 서버.

프런트엔드에 빌드 단계를 두지 않았으므로(순수 HTML/JS + 동봉한 roslib) 번들러도
node 도 필요 없다. 파일을 그대로 내려주기만 하면 된다.

    ros2 run crazyflie_fms web_server --ros-args -p port:=8080
"""
import functools
import http.server
import os
import socketserver

from ament_index_python.packages import get_package_share_directory

DEFAULT_PORT = 8080


class _QuietHandler(http.server.SimpleHTTPRequestHandler):
    """요청마다 stderr 에 한 줄씩 찍는 기본 동작은 launch 로그를 덮어버린다."""

    def log_message(self, fmt, *args):
        pass


def main():
    port = int(os.environ.get('FMS_WEB_PORT', DEFAULT_PORT))
    web_dir = os.environ.get('FMS_WEB_DIR') or os.path.join(
        get_package_share_directory('crazyflie_fms'), 'web')

    handler = functools.partial(_QuietHandler, directory=web_dir)
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(('0.0.0.0', port), handler) as httpd:
        print(f'[fms web] {web_dir} → http://localhost:{port}', flush=True)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
