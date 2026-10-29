#!/usr/bin/env python3
"""No-dependency localhost server smoke; invoked by CTest only for the demo target."""
import json
import re
import socket
import tempfile
from pathlib import Path
import subprocess
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from urllib.parse import urlencode


def get(url, method="GET", data=None, headers=None):
    try:
        with urlopen(Request(url, method=method, data=data, headers=headers or {}), timeout=12) as response:
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
        assert json.loads(payload)["differential"] is True
        assert json.loads(payload)["mecanum"] is True
        assert headers.get("X-Content-Type-Options") == "nosniff"
        for resource, content in [("/", "Planning Lab"),
                                  ("/style.css", "map-wrap"),
                                  ("/app.js", "replan"),
                                  ("/map_editor.js", "MapEditor"),
                                  ("/map_geometry.js", "MapGeometry")]:
            code, _, body = get(base + resource)
            assert code == 200 and content in body, resource
        # Browser bootstrap contract: app.js looks controls up by element id, so a
        # missing element aborts listener registration and the page never leaves the
        # "Preparing map…" overlay. Check the served markup against the served script.
        _, _, page = get(base + "/")
        _, _, script = get(base + "/app.js")
        referenced = set(re.findall(r"\$\('([A-Za-z0-9_]+)'\)", script))
        defined = set(re.findall(r'id="([A-Za-z0-9_]+)"', page))
        assert len(referenced) > 20 and len(defined) > 20
        missing = referenced - defined
        assert not missing, f"index.html lacks element ids used by app.js: {sorted(missing)}"
        assert re.search(r'id="inflationMeters"[^>]*step="0\.05"', page), (
            "footprint margin spinner must step by 0.05 m"
        )
        assert 'id="refine"' in page and "Strict post-path refinement" in page
        assert 'id="steeringSmooth"' in page and "Steering-aware smoothness" in page
        assert re.search(r'<select id="maxAccepted" disabled>', page)
        assert re.search(r'<option value="8" selected>', page)
        assert '<option value="12">' in page and '<option value="16">' in page
        assert "params.set('maxAccepted',$('maxAccepted').value)" in script
        assert "$('maxAccepted').disabled=!ackermann||!$('refine').checked" in script
        assert "$('maxAccepted').addEventListener('change',invalidateRoute)" in script
        assert re.search(r'id="steeringWeight"[^>]*value="1\.0"[^>]*disabled', page)
        assert "params.set('steeringSmooth'," in script
        assert "params.set('steeringWeight'," in script
        assert "$('steeringSmooth').disabled=!canUseSteering" in script
        assert "$('steeringWeight').disabled=!canUseSteering" in script
        assert 'id="curvatureVariation"' in page and 'id="steeringReversals"' in page
        assert 'function steeringMetrics(path)' in script
        assert 'id="rsDistance"' in page and "RS distance heuristic" in page
        assert "['ompl','rsDistance','refine'" in script
        assert "$('ompl').checked=!!caps.ompl" in script
        assert "mecanum||!$('refine').checked" in script, (
            "mecanum only disables curved CSC, not strict holonomic refinement"
        )
        # Every button on this page is wired by id, so a dropped opening tag (which
        # leaves an orphan </button> and no element) is a markup defect too.
        tags = re.findall(r"<(/?)button\b([^>]*)>", page)
        assert all('id="' in attributes for slash, attributes in tags if slash == ""), \
            "every <button> must carry an id the script can look up"
        depth = 0
        for slash, _ in tags:
            depth += -1 if slash else 1
            assert depth >= 0, "orphan </button> in the served page"
        assert depth == 0, "unclosed <button> in the served page"
        for control in ["planButton", "exportButton", "budget", "limit", "map", "overlay", "message"]:
            assert f'id="{control}"' in page, control
        code, _, body = get(base + "/api/plan?scenario=reverse&startx=10&starty=15"
                            "&startyaw=0&goalx=6&goaly=15&goalyaw=0")
        result = json.loads(body)
        assert code == 200 and result["original"]["status"] == "Success"
        assert result["original"]["validated"] is True
        assert result["original"]["heuristicCertifiedAdmissible"] is False
        assert result["rsDistanceHeuristic"] is False
        assert result["refinementOptions"]["maxAttempts"] == 160
        assert result["refinementOptions"]["maxAccepted"] == 8
        assert result["refinementOptions"]["maxTotalPoseChecks"] == 200000
        for accepted in (8, 12, 16):
            code, _, trial_body = get(
                base + f"/api/plan?scenario=open&refine=1&maxAccepted={accepted}")
            trial = json.loads(trial_body)
            assert code == 200 and trial["original"]["status"] == "Success", trial_body
            assert trial["refinement"]["path"]["status"] == "Success", trial_body
            assert trial["refinementOptions"]["maxAttempts"] == 160
            assert trial["refinementOptions"]["maxAccepted"] == accepted
            assert trial["refinementOptions"]["maxTotalPoseChecks"] == 200000
            assert trial["refinement"]["accepted"] <= accepted
        for invalid_accepted in (
            "scenario=open&refine=1&maxAccepted=0",
            "scenario=open&refine=1&maxAccepted=10",
            "scenario=open&refine=1&maxAccepted=17",
            "scenario=open&refine=1&maxAccepted=abc",
            "scenario=open&refine=1&maxAccepted=12.0",
            "scenario=open&refine=1&maxAccepted=-1",
            "scenario=open&refine=1&maxAccepted=012",
            "scenario=open&maxAccepted=16",
            "scenario=open&model=differential&refine=1&maxAccepted=12",
            "scenario=open&model=mecanum&refine=1&maxAccepted=16",
        ):
            code, _, rejected_body = get(base + "/api/plan?" + invalid_accepted)
            assert code == 400 and "error" in json.loads(rejected_body), invalid_accepted
        assert result["refinementOptions"]["steeringSmoothness"] is False
        assert result["refinementOptions"]["steeringWeightMetersSquared"] == 0.0
        code, _, body = get(
            base + "/api/plan?scenario=open&refine=1&steeringSmooth=1&steeringWeight=2.5")
        steering = json.loads(body)
        assert code == 200 and steering["original"]["status"] == "Success", body
        assert steering["refinement"]["path"]["status"] == "Success", body
        assert steering["refinementOptions"]["maxAttempts"] == 160
        assert steering["refinementOptions"]["maxAccepted"] == 8
        assert steering["refinementOptions"]["maxTotalPoseChecks"] == 200000
        assert steering["refinementOptions"]["steeringSmoothness"] is True
        assert steering["refinementOptions"]["steeringWeightMetersSquared"] == 2.5
        code, _, disabled_body = get(
            base + "/api/plan?scenario=open&refine=1&steeringSmooth=0&steeringWeight=2.5")
        disabled_steering = json.loads(disabled_body)
        assert code == 200
        assert disabled_steering["refinementOptions"]["steeringSmoothness"] is False
        assert disabled_steering["refinementOptions"]["steeringWeightMetersSquared"] == 0.0
        for invalid_steering in (
            "scenario=open&steeringSmooth=1",
            "scenario=open&model=differential&refine=1&steeringSmooth=1",
            "scenario=open&model=mecanum&refine=1&steeringSmooth=1",
            "scenario=open&refine=1&steeringSmooth=2",
            "scenario=open&refine=1&steeringSmooth=1&steeringWeight=0",
            "scenario=open&refine=1&steeringSmooth=1&steeringWeight=-0.1",
            "scenario=open&refine=1&steeringSmooth=1&steeringWeight=101",
            "scenario=open&refine=1&steeringSmooth=1&steeringWeight=nan",
        ):
            status, _, invalid_body = get(base + "/api/plan?" + invalid_steering)
            assert status == 400 and "error" in json.loads(invalid_body), invalid_steering
        for invalid_rs in (
            "scenario=open&rsDistance=1&ompl=0",
            "scenario=open&rsDistance=1&ompl=1&performance=0",
        ):
            status, _, invalid_body = get(base + "/api/plan?" + invalid_rs)
            assert status == 400 and "error" in json.loads(invalid_body), invalid_rs
        assert result["footprintInflationMeters"] == 0.0
        for enabled, configured, expected in (
            ("1", "0.05", 0.05), ("1", "0.1", 0.1), ("0", "0.1", 0.0)
        ):
            code, _, margin_body = get(
                base + "/api/plan?scenario=open&inflate=" + enabled
                + "&inflationMeters=" + configured)
            parsed_margin = json.loads(margin_body)
            assert code == 200 and parsed_margin["original"]["status"] == "Success", margin_body
            assert abs(parsed_margin["footprintInflationMeters"] - expected) < 1e-12
            assert parsed_margin["original"]["validated"] is True
        for invalid_margin in (
            "inflate=1&inflationMeters=-0.01",
            "inflate=1&inflationMeters=11",
            "inflate=1&inflationMeters=nan",
            "inflate=2&inflationMeters=0.05",
        ):
            code, _, margin_body = get(base + "/api/plan?scenario=open&" + invalid_margin)
            assert code == 400 and "error" in json.loads(margin_body), invalid_margin
        assert result["original"]["reverseLength"] >= 3.5
        code, _, body = get(base + "/api/plan?scenario=reverse&performance=0")
        certified = json.loads(body)
        assert code == 200 and certified["original"]["status"] == "Success"
        assert certified["original"]["heuristicCertifiedAdmissible"] is True
        code, _, body = get(base + "/api/plan?scenario=warehouse&performance=1")
        warehouse = json.loads(body)
        assert code == 200 and warehouse["original"]["status"] == "Success"
        assert warehouse["mapRuns"]
        # Differential-drive post-path refinement is available through the
        # same HTTP flags as Ackermann, and must preserve strict validation.
        code, _, body = get(base + "/api/plan?scenario=open&model=differential"
                            "&refine=1&curved=1")
        smooth = json.loads(body)
        assert code == 200 and smooth["original"]["status"] == "Success", body
        assert smooth["refinement"] is not None
        assert smooth["refinement"]["path"] is not None
        assert smooth["refinement"]["path"]["status"] == "Success"
        assert smooth["refinement"]["path"]["cost"] <= smooth["original"]["cost"] + 1e-8
        assert (
            smooth["refinement"]["path"]["collisionEvidence"]
            == "FullFootprintBodyMotionAdaptiveSampled"
        )
        code, _, body = get(base + "/api/plan?scenario=open&model=mecanum&refine=1"
                            "&inflate=1&inflationMeters=0.05")
        mecanum = json.loads(body)
        assert code == 200 and mecanum["original"]["status"] == "Success", body
        assert mecanum["refinement"] is not None
        assert mecanum["refinement"]["path"]["status"] == "Success"
        assert mecanum["refinement"]["path"]["validated"] is True
        assert mecanum["refinement"]["path"]["cost"] <= mecanum["original"]["cost"] + 1e-8
        # Work budgets are request-tunable. Hitting one is a limit, never a NoPath.
        # Keep this starvation check on certified Euclidean ordering: the
        # new default obstacle-grid can solve the warehouse with fewer nodes.
        code, _, body = get(base + "/api/plan?scenario=warehouse&performance=0&budget=2000")
        starved = json.loads(body)
        assert code == 200 and starved["original"]["status"] == "BudgetExceeded", body
        assert starved["limits"]["maxExpansions"] == 2000
        assert starved["original"]["expanded"] >= 2000
        assert starved["original"]["collisionEvidence"] == "None"
        code, _, body = get(base + "/api/plan?scenario=warehouse&budget=60000")
        funded = json.loads(body)
        assert code == 200 and funded["original"]["status"] == "Success"
        assert funded["limits"]["maxExpansions"] == 60000
        assert funded["limits"]["maxGeneratedNodes"] >= 60000
        assert funded["original"]["collisionEvidence"] == "FullFootprintBodyMotionAdaptiveSampled"
        for invalid_budget in ["scenario=warehouse&budget=0", "scenario=warehouse&budget=1000001",
                               "scenario=warehouse&budget=abc"]:
            code, _, body = get(base + "/api/plan?" + invalid_budget)
            assert code == 400 and "error" in json.loads(body), invalid_budget
        for invalid in ["scenario=unlisted", "scenario=open&scenario=open",
                        "scenario=open&startx=nan&starty=15&startyaw=0",
                        "scenario=open&ompl=1"] if not json.loads(payload)["ompl"] else [
                            "scenario=unlisted", "scenario=open&scenario=open",
                            "scenario=open&startx=nan&starty=15&startyaw=0"]:
            code, _, body = get(base + "/api/plan?" + invalid)
            assert code == 400 and "error" in json.loads(body), invalid
        # Planning POST requires a bounded URL-encoded body, not a naked POST.
        code, _, body = get(base + "/api/plan", method="POST")
        assert code == 400 and "error" in json.loads(body)
        code, _, body = get(base + "/", method="POST")
        assert code == 405 and "error" in json.loads(body)
        def post(fields):
            payload = urlencode(fields).encode()
            return get(base + "/api/plan", method="POST", data=payload,
                       headers={"Content-Type": "application/x-www-form-urlencoded"})
        # Differential-drive mode is served by the same C++ planner, with
        # model-specific geometry and nonzero-cost rotation in place.
        code, _, body = post({"scenario": "open", "model": "differential",
                               "startx": "10", "starty": "15", "startyaw": "0",
                               "goalx": "10", "goaly": "15", "goalyaw": "90"})
        spin = json.loads(body)
        assert code == 200 and spin["model"] == "differential"
        assert spin["footprint"] == [-.35, .35, .25]
        assert spin["original"]["status"] == "Success"
        assert spin["original"]["validated"] is True
        assert spin["original"]["cost"] > 0
        assert spin["original"]["collisionEvidence"] == "FullFootprintBodyMotionAdaptiveSampled"
        assert any(sec["kind"] == "rotation" and sec["length"] == 0 and
                   len(sec["points"]) > 2 and sec["turnRadians"] != 0
                   for sec in spin["original"]["sections"])
        code, _, body = post({"scenario": "open", "model": "mecanum",
                              "startx": "10", "starty": "15", "startyaw": "0",
                              "goalx": "10", "goaly": "17", "goalyaw": "0"})
        lateral = json.loads(body)
        assert code == 200 and lateral["model"] == "mecanum"
        assert lateral["original"]["status"] == "Success"
        assert lateral["original"]["validated"] is True
        assert lateral["original"]["actuatorFeasible"] is False
        assert any(sec["kind"] == "holonomic" and abs(sec["lateralMeters"]) > 0
                   and sec["gear"] == "holonomic"
                   for sec in lateral["original"]["sections"])
        code, _, body = post({"scenario":"open", "model":"mecanum", "ompl":"1"})
        assert code == 400
        code, _, body = post({"scenario":"open", "model":"differential", "ompl":"1"})
        assert code == 400 and "error" in json.loads(body)
        code, _, body = post({"scenario":"open", "model":"unsupported"})
        assert code == 400 and "error" in json.loads(body)
        base_route = json.loads(post({"scenario":"reverse"})[2])
        assert base_route["original"]["status"] == "Success"
        assert base_route["revision"] == 1
        # Cell (40,60) intersects the start footprint at x=10,y=15.
        cell = 60*200+40
        code, _, body = post({"scenario":"reverse", "edits":f"{cell}:100",
                              "editRevision":"1"})
        obstacle = json.loads(body)
        assert code == 200 and obstacle["revision"] == 2
        assert obstacle["original"]["status"] == "StartInCollision"
        assert obstacle["original"]["collisionEvidence"] == "None"
        assert [40,60,1,100] in obstacle["mapRuns"]
        # The previous validated path was pinned to its original revision.
        assert base_route["original"]["mapRevision"] == 1
        code, _, body = post({"scenario":"reverse", "edits":"", "editRevision":"2"})
        restored = json.loads(body)
        assert code == 200 and restored["revision"] == 3
        assert restored["original"]["status"] == "Success"
        assert restored["original"]["mapRevision"] == 3
        for illegal in ({"edits":f"{cell}:100"},
                        {"editRevision":"1","edits":f"{cell}:100,{cell}:0"},
                        {"editRevision":"1","edits":"999999:100"},
                        {"editRevision":"1","edits":"0:25"},
                        {"editRevision":"NaN","edits":"0:100"}):
            code, _, body = post({"scenario":"reverse", **illegal})
            assert code == 400 and "error" in json.loads(body), illegal
        code, _, body = get(base + "/api/plan", method="POST",
                             data=b"scenario=reverse",
                             headers={"Content-Type":"text/plain"})
        assert code == 400 and "error" in json.loads(body)

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
        funded_cli = subprocess.run([exe, "--once", "--scenario", "warehouse", "--budget", "60000"],
                                    capture_output=True, text=True)
        assert funded_cli.returncode == 0
        assert json.loads(funded_cli.stdout)["limits"]["maxExpansions"] == 60000
        steering_cli = subprocess.run(
            [exe, "--once", "--scenario", "open", "--refine", "--steering-weight", "2.5"],
            capture_output=True, text=True, timeout=20)
        assert steering_cli.returncode == 0, steering_cli.stderr
        steering_cli_data = json.loads(steering_cli.stdout)
        assert steering_cli_data["refinementOptions"]["steeringSmoothness"] is True
        assert steering_cli_data["refinementOptions"]["steeringWeightMetersSquared"] == 2.5
        accepted_cli = subprocess.run(
            [exe, "--once", "--scenario", "open", "--refine", "--max-accepted", "16"],
            capture_output=True, text=True, timeout=20)
        assert accepted_cli.returncode == 0, accepted_cli.stderr
        accepted_data = json.loads(accepted_cli.stdout)
        assert accepted_data["refinementOptions"]["maxAccepted"] == 16
        assert accepted_data["refinementOptions"]["maxAttempts"] == 160
        assert accepted_data["refinementOptions"]["maxTotalPoseChecks"] == 200000
        invalid_accepted_cli = subprocess.run(
            [exe, "--once", "--scenario", "open", "--refine", "--max-accepted", "10"],
            capture_output=True, text=True, timeout=20)
        assert invalid_accepted_cli.returncode != 0
        assert "Accepted replacements must be 8, 12 or 16" in invalid_accepted_cli.stderr
        invalid_budget = subprocess.run([exe, "--once", "--scenario", "warehouse", "--budget", "7"],
                                        capture_output=True, text=True)
        assert invalid_budget.returncode != 0
        assert "Budget must be an integer" in invalid_budget.stderr
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
