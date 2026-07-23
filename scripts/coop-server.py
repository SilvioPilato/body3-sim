#!/usr/bin/env python3
"""Static server for local wasm testing. Sends the COOP/COEP headers that
SharedArrayBuffer (and thus macroquad's threaded wasm) requires — a plain
`python -m http.server` does not, so the app cannot run under it. Mirrors the
production `_headers` file and serves .wasm as application/wasm.

Usage: python scripts/coop-server.py [dir] [port]   (defaults: dist 8080)
"""
import http.server, socketserver, sys, os

directory = sys.argv[1] if len(sys.argv) > 1 else "dist"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080
os.chdir(directory)

class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map,
                      ".wasm": "application/wasm", ".js": "text/javascript"}
    def end_headers(self):
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        super().end_headers()

with socketserver.TCPServer(("127.0.0.1", port), Handler) as httpd:
    print(f"serving {directory} on http://127.0.0.1:{port}  (ctrl-c to stop)", flush=True)
    httpd.serve_forever()
