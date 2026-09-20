"""Unit tests for the dashboard HTTP server and REST API endpoints."""

import json
import socketserver
import threading
import time
import urllib.request

from maze_solver.dashboard.server import DashboardHandler


def test_dashboard_api_endpoints() -> None:
    """Test dashboard HTTP endpoints for maze generation, solving, and simulation."""
    # Start server on an ephemeral random free port
    with socketserver.TCPServer(("127.0.0.1", 0), DashboardHandler) as httpd:
        port = httpd.server_address[1]
        server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
        server_thread.start()
        time.sleep(0.1)

        base_url = f"http://127.0.0.1:{port}"

        # 1. Test GET / (index.html)
        req = urllib.request.Request(f"{base_url}/")
        with urllib.request.urlopen(req) as resp:
            assert resp.status == 200
            html_content = resp.read().decode("utf-8")
            assert "MMRC26 MICROMOUSE COMMAND DECK" in html_content
            assert "Readiness Audit" in html_content

        # 2. Test GET /api/maze?seed=42
        req = urllib.request.Request(f"{base_url}/api/maze?seed=42")
        with urllib.request.urlopen(req) as resp:
            assert resp.status == 200
            maze_data = json.loads(resp.read().decode("utf-8"))
            assert maze_data["seed"] == 42
            assert len(maze_data["horizontal_walls"]) == 11
            assert len(maze_data["vertical_walls"]) == 10

        # 3. Test POST /api/solve
        solve_payload = json.dumps(
            {
                "horizontal_walls": maze_data["horizontal_walls"],
                "vertical_walls": maze_data["vertical_walls"],
            }
        ).encode("utf-8")
        req = urllib.request.Request(
            f"{base_url}/api/solve",
            data=solve_payload,
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req) as resp:
            assert resp.status == 200
            solve_data = json.loads(resp.read().decode("utf-8"))
            assert len(solve_data["distance_map"]) == 10
            assert len(solve_data["path_coords"]) > 0
            assert solve_data["time_suction_sec"] < solve_data["time_baseline_sec"]

        # 4. Test POST /api/simulate
        sim_payload = json.dumps(
            {"seed": 42, "use_suction": True, "enable_return_trip": True}
        ).encode("utf-8")
        req = urllib.request.Request(
            f"{base_url}/api/simulate",
            data=sim_payload,
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req) as resp:
            assert resp.status == 200
            sim_data = json.loads(resp.read().decode("utf-8"))
            assert sim_data["total_runs"] > 0
            assert sim_data["official_time"] > 0
            assert sim_data["final_score"] > 0
            assert len(sim_data["runs"]) > 0

        httpd.shutdown()
