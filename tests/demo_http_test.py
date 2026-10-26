#!/usr/bin/env python3
"""No-dependency localhost server smoke; invoked by CTest only for the demo target."""
import json
import socket
import subprocess
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def get(url, method="GET"):
    try:
        with urlopen(Request(url, method=method), timeout=8) as response:
            return response.status, response.headers, response.read().decode()
    except HTTPError as exc:
        return exc.code, exc.headers, exc.read().decode()


def main(exe):
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    proc = subprocess.Popen([exe, "--port", str(port)], stdout=subprocess.DEVNULL,
                            stderr=subprocess.PIPE)
    base = f"http://127.0.0.1:{port}"
    try:
        for _ in range(60):
            if proc.poll() is not None:
                raise AssertionError("Demo server quit before accepting connections")
            try:
                code, headers, payload = get(base + "/api/capabilities")
                break
            except (URLError, ConnectionError):
                time.sleep(0.05)
        else:
            raise AssertionError("Demo server never became ready")
        assert code == 200 and isinstance(json.loads(payload)["ompl"], bool)
        assert headers.get("X-Content-Type-Options") == "nosniff"
        for resource, content in [("/", "Planning Lab"),
                                  ("/style.css", "map-wrap"),
                                  ("/app.js", "replan")]:
            code, _, body = get(base + resource)
            assert code == 200 and content in body, resource
        code, _, body = get(base + "/api/plan?scenario=reverse&startx=10&starty=15"
                            "&startyaw=0&goalx=6&goaly=15&goalyaw=0")
        result = json.loads(body)
        assert code == 200 and result["original"]["status"] == "Success"
        assert result["original"]["validated"] is True
        assert result["original"]["reverseLength"] >= 3.5
        code, _, body = get(base + "/api/plan?scenario=warehouse&performance=1")
        warehouse = json.loads(body)
        assert code == 200 and warehouse["original"]["status"] == "Success"
        assert warehouse["mapRuns"]
        for invalid in ["scenario=unlisted", "scenario=open&scenario=open",
                        "scenario=open&startx=nan&starty=15&startyaw=0",
                        "scenario=open&ompl=1"] if not json.loads(payload)["ompl"] else [
                            "scenario=unlisted", "scenario=open&scenario=open",
                            "scenario=open&startx=nan&starty=15&startyaw=0"]:
            code, _, body = get(base + "/api/plan?" + invalid)
            assert code == 400 and "error" in json.loads(body), invalid
        code, _, body = get(base + "/api/plan", method="POST")
        assert code == 405 and "error" in json.loads(body)
        print("DEMO HTTP PASS")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3)


if __name__ == "__main__":
    main(sys.argv[1])
