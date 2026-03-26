#!/usr/bin/env python3
"""
server.py — Kerbin Map dev server

Serves the web viewer and provides a /api/export endpoint that regenerates
the nav CSVs from generate_static_nav_csv.py without needing KSP running.

Usage:
    cd "f:\\Kerbal Space Program\\Ships\\Script\\Integrated Flight Computer\\tools\\kerbin_map"
    python server.py
    Open: http://localhost:8080

Optional port argument:
    python server.py 9090
"""

import sys
import json
import importlib.util
from pathlib import Path
from http.server import HTTPServer, SimpleHTTPRequestHandler

HERE       = Path(__file__).parent
WEB_DIR    = HERE / "web"
GEN_SCRIPT = HERE / "python" / "generate_static_nav_csv.py"


def _run_export() -> str:
    """Import and run generate_static_nav_csv.main(), return summary string."""
    spec = importlib.util.spec_from_file_location("generate_static_nav_csv", GEN_SCRIPT)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.main()
    return "Export complete"


class KerbinMapHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(WEB_DIR), **kwargs)

    def do_POST(self):
        if self.path != "/api/export":
            self.send_error(404)
            return
        try:
            msg  = _run_export()
            body = json.dumps({"ok": True, "message": msg}).encode()
            self.send_response(200)
        except Exception as exc:
            body = json.dumps({"ok": False, "message": str(exc)}).encode()
            self.send_response(500)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        print(f"  {self.address_string()} {fmt % args}")


if __name__ == "__main__":
    port   = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    server = HTTPServer(("", port), KerbinMapHandler)
    print(f"Kerbin Map  →  http://localhost:{port}")
    print(f"Static root:   {WEB_DIR}")
    print(f"Export script: {GEN_SCRIPT}")
    print("Ctrl-C to stop.\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
