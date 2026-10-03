"""Local static server for ARES-6 with caching disabled (edits are always picked up).
Usage:  python tools/serve.py [port]   -> http://127.0.0.1:8642/"""
import functools
import http.server
import os
import sys


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, must-revalidate")
        self.send_header("Expires", "0")
        super().end_headers()


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8642
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    handler = functools.partial(NoCacheHandler, directory=root)
    print(f"ARES-6 -> http://127.0.0.1:{port}/   (test suite: /?test)", flush=True)
    http.server.ThreadingHTTPServer(("127.0.0.1", port), handler).serve_forever()
