#!/usr/bin/env python3
"""No-dependency localhost server smoke; invoked by CTest only for the demo target."""
import json
import socket
import tempfile
from pathlib import Path
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
                                  ("/app.js", "replan"),
                                  ("/map_geometry.js", "MapGeometry")]:
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
        with tempfile.TemporaryDirectory() as directory:
            pgm = Path(directory) / "open-field.pgm"
            pgm.write_bytes(b"P5\n160 120\n255\n" + bytes([255]) * (160 * 120))
            out = Path(directory) / "plan.json"
            once = subprocess.run([exe, "--once", "--map", str(pgm),
                                   "--output", str(out)], capture_output=True,
                                  text=True, timeout=20)
            assert once.returncode == 0, once.stderr
            saved = json.loads(out.read_text())
            assert saved["original"]["status"] == "Success"
            assert saved["width"] == 160 and saved["height"] == 120
            assert saved["sourceId"] == "imported-pgm"
            assert saved["original"]["collisionEvidence"] == "FullFootprintBodyMotionAdaptiveSampled"
            assert saved["original"]["actuatorFeasible"] is False
            malformed = Path(directory) / "bad.pgm"
            malformed.write_bytes(b"P5\n1200 1\n255\n")
            rejected = subprocess.run([exe, "--once", "--map", str(malformed)],
                                      capture_output=True, text=True, timeout=10)
            assert rejected.returncode != 0
            with socket.socket() as sock:
                sock.bind(("127.0.0.1", 0))
                custom_port = sock.getsockname()[1]
            child = subprocess.Popen([exe, "--port", str(custom_port), "--map", str(pgm)],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            try:
                origin = f"http://127.0.0.1:{custom_port}"
                for _ in range(60):
                    try:
                        state = json.loads(get(origin + "/api/capabilities")[2])
                        break
                    except (URLError, ConnectionError):
                        time.sleep(.05)
                else:
                    raise AssertionError("Imported-map server unavailable")
                assert state["customMap"] is True
                assert state["mapWidthMeters"] == 40
                code, _, response = get(origin + "/api/plan?scenario=custom")
                assert code == 200
                imported = json.loads(response)
                assert imported["original"]["status"] == "Success"
                assert imported["width"] == 160
            finally:
                child.terminate()
                child.wait(timeout=3)
        # A real map-server YAML with nonzero translation and 90-degree yaw:
        # test both the CLI world-frame output and the hosted map transform.
        yaml_path = Path(__file__).resolve().parents[1] / "apps/demo/maps/sample.yaml"
        rotated = subprocess.run([exe, "--once", "--map-yaml", str(yaml_path)],
                                 capture_output=True, text=True, timeout=25)
        assert rotated.returncode == 0, rotated.stderr
        rotated_data = json.loads(rotated.stdout)
        assert rotated_data["original"]["status"] == "Success"
        assert rotated_data["sourceId"] == "ros-map-yaml"
        assert rotated_data["origin"][:2] == [5, -2]
        assert abs(rotated_data["start"][0] + 10) < 1e-8
        assert abs(rotated_data["start"][1] - 6) < 1e-8
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            yaml_port = sock.getsockname()[1]
        child = subprocess.Popen([exe, "--port", str(yaml_port),
                                  "--map-yaml", str(yaml_path)],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        try:
            origin = f"http://127.0.0.1:{yaml_port}"
            for _ in range(60):
                try:
                    state = json.loads(get(origin + "/api/capabilities")[2])
                    break
                except (URLError, ConnectionError):
                    time.sleep(.05)
            else:
                raise AssertionError("YAML map server unavailable")
            assert state["customMap"] is True
            assert abs(state["origin"][2] - 1.5707963267948966) < 1e-10
            code, _, response = get(origin + "/api/plan?scenario=custom")
            assert code == 200
            assert json.loads(response)["original"]["status"] == "Success"
        finally:
            child.terminate()
            child.wait(timeout=3)
        invalid = subprocess.run([exe, "--once", "--map-yaml", str(yaml_path),
                                  "--resolution", "1.0"], capture_output=True, text=True)
        assert invalid.returncode != 0
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
