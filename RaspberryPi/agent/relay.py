"""Small LAN relay for sending Agent API requests through the host computer.

The Raspberry Pi sends OpenAI-compatible requests to this process. The relay
adds the selected local API.md credentials and forwards the request upstream.
It intentionally does not log authorization headers or request bodies.
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .config import load_api_config


class RelayHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def log_message(self, format, *args):
        # Keep the log useful without ever printing headers or body contents.
        print(f"[Relay] {self.address_string()} {format % args}", flush=True)

    def do_POST(self):  # noqa: N802 - required by BaseHTTPRequestHandler
        server = self.server
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > server.max_body_bytes:
            self._send_json(413, {"error": "request body too large or empty"})
            return
        body = self.rfile.read(length)
        target = server.upstream_base_url.rstrip("/") + self.path
        request = urllib.request.Request(
            target,
            data=body,
            method="POST",
            headers={
                "Content-Type": self.headers.get("Content-Type", "application/json"),
                "Accept": self.headers.get("Accept", "application/json"),
                "Authorization": f"Bearer {server.upstream_api_key}",
            },
        )
        started = time.monotonic()
        try:
            with urllib.request.urlopen(request, timeout=server.timeout_s) as response:
                payload = response.read(server.max_body_bytes + 1)
                if len(payload) > server.max_body_bytes:
                    self._send_json(413, {"error": "upstream response too large"})
                    return
                status = response.status
                content_type = response.headers.get("Content-Type", "application/json")
        except urllib.error.HTTPError as exc:
            payload = exc.read(server.max_body_bytes + 1)
            status = exc.code
            content_type = exc.headers.get("Content-Type", "application/json")
        except Exception as exc:
            print(f"[Relay] upstream error: {type(exc).__name__}: {exc}", flush=True)
            self._send_json(502, {"error": "upstream API unavailable"})
            return

        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)
        elapsed = time.monotonic() - started
        print(f"[Relay] POST {self.path} -> {status} ({elapsed:.2f}s)", flush=True)

    def _send_json(self, status, value):
        payload = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


def build_parser():
    parser = argparse.ArgumentParser(description="Uniforest LAN API relay")
    parser.add_argument("--host", default="0.0.0.0", help="监听地址")
    parser.add_argument("--port", type=int, default=8765, help="监听端口")
    parser.add_argument("--profile", default="APIFUN gpt",
                        help="API.md 上游配置分组，默认 APIFUN gpt")
    parser.add_argument("--api-config", default=None,
                        help="API.md 路径")
    parser.add_argument("--timeout", type=float, default=90.0)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    config = load_api_config(args.api_config, profile=args.profile)
    if not config["base_url"] or not config["api_key"]:
        raise SystemExit(f"API 配置不完整: profile={config['profile']}")
    server = ThreadingHTTPServer((args.host, args.port), RelayHandler)
    server.upstream_base_url = config["base_url"]
    server.upstream_api_key = config["api_key"]
    server.timeout_s = args.timeout
    server.max_body_bytes = 25 * 1024 * 1024
    print(f"[Relay] listening on {args.host}:{args.port}", flush=True)
    print(f"[Relay] upstream profile={config['profile']} base_url={config['base_url']}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Relay] stopped", flush=True)
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
