"""Lightweight HTTP server serving the interactive web dashboard and REST API."""

import http.server
import json
import os
import socketserver
import urllib.parse
from typing import Any

from maze_solver.core.diagonal_planner import (
    evaluate_smoothed_trajectory_time,
    smooth_path_to_diagonals,
)
from maze_solver.core.floodfill import compute_distance_map
from maze_solver.core.kinematics import KinematicProfile
from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.planner import plan_turn_weighted_path
from maze_solver.core.types import Cell, Direction, RobotState
from maze_solver.simulator.match_simulator import MatchSimulator
from maze_solver.simulator.maze_generator import generate_island_maze


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    """Custom HTTP handler serving dashboard static assets and API endpoints."""

    def do_GET(self) -> None:
        """Handle GET requests for static assets and API data."""
        parsed_url = urllib.parse.urlparse(self.path)
        path = parsed_url.path

        if path in ("", "/"):
            self._serve_static_file("index.html", "text/html")
            return

        if path == "/api/maze":
            query_params = urllib.parse.parse_qs(parsed_url.query)
            seed = int(query_params.get("seed", ["42"])[0])
            grid = generate_island_maze(seed=seed)
            self._send_json(
                {
                    "seed": seed,
                    "rows": grid.rows,
                    "cols": grid.cols,
                    "horizontal_walls": grid.horizontal_walls,
                    "vertical_walls": grid.vertical_walls,
                }
            )
            return

        # Fallback to static directory
        static_dir = os.path.join(os.path.dirname(__file__), "static")
        safe_path = os.path.normpath(os.path.join(static_dir, path.lstrip("/")))
        if safe_path.startswith(static_dir) and os.path.exists(safe_path):
            content_type = (
                "text/html" if safe_path.endswith(".html") else "application/octet-stream"
            )
            if safe_path.endswith(".js"):
                content_type = "application/javascript"
            elif safe_path.endswith(".css"):
                content_type = "text/css"
            self._serve_file(safe_path, content_type)
            return

        self.send_error(404, "Not Found")

    def do_POST(self) -> None:
        """Handle POST requests for algorithmic calculations and simulation."""
        parsed_url = urllib.parse.urlparse(self.path)
        path = parsed_url.path

        content_len = int(self.headers.get("Content-Length", 0))
        post_data = self.rfile.read(content_len) if content_len > 0 else b"{}"

        try:
            body = json.loads(post_data.decode("utf-8"))
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON payload")
            return

        if path == "/api/solve":
            self._handle_solve(body)
            return

        if path == "/api/simulate":
            self._handle_simulate(body)
            return

        self.send_error(404, "Endpoint not found")

    def _handle_solve(self, body: dict[str, Any]) -> None:
        """Calculate distance maps, optimal turn-weighted paths, and diagonal trajectories."""
        grid = MazeGrid()
        if "horizontal_walls" in body and "vertical_walls" in body:
            grid.horizontal_walls = body["horizontal_walls"]
            grid.vertical_walls = body["vertical_walls"]
        else:
            seed = int(body.get("seed", 42))
            grid = generate_island_maze(seed=seed)

        # 1. Distance map
        dist_map = compute_distance_map(grid)

        # 2. Optimal turn-weighted path
        start_state = RobotState(Cell(9, 0), Direction.NORTH)
        commands = plan_turn_weighted_path(grid, start_state)

        # Reconstruct path coordinates
        curr = start_state
        path_coords: list[dict[str, int]] = [{"row": curr.cell.row, "col": curr.cell.col}]
        for cmd in commands:
            if cmd.name == "FORWARD":
                next_cell = curr.cell.neighbor(curr.heading)
                curr = RobotState(next_cell, curr.heading)
                path_coords.append({"row": curr.cell.row, "col": curr.cell.col})
            elif cmd.name == "TURN_LEFT":
                curr = RobotState(curr.cell, curr.heading.turn_left())
            elif cmd.name == "TURN_RIGHT":
                curr = RobotState(curr.cell, curr.heading.turn_right())
            elif cmd.name == "TURN_AROUND":
                curr = RobotState(curr.cell, curr.heading.turn_around())

        # 3. Diagonal smoothing
        segments = smooth_path_to_diagonals(commands, grid, start_state)
        smoothed_segments_data = [
            {
                "type": seg.segment_type.value,
                "distance_m": round(seg.distance_m, 3),
                "units_count": seg.units_count,
            }
            for seg in segments
        ]

        # 4. Kinematics evaluation
        t_base = evaluate_smoothed_trajectory_time(
            segments, KinematicProfile(suction_multiplier=0.0)
        )
        t_suction = evaluate_smoothed_trajectory_time(
            segments, KinematicProfile(suction_multiplier=3.0)
        )

        self._send_json(
            {
                "distance_map": dist_map,
                "commands": [c.name for c in commands],
                "path_coords": path_coords,
                "smoothed_segments": smoothed_segments_data,
                "time_baseline_sec": round(t_base, 3),
                "time_suction_sec": round(t_suction, 3),
            }
        )

    def _handle_simulate(self, body: dict[str, Any]) -> None:
        """Run full 480-second match tournament simulation."""
        seed = int(body.get("seed", 42))
        use_suction = bool(body.get("use_suction", True))
        enable_return_trip = bool(body.get("enable_return_trip", True))

        grid = generate_island_maze(seed=seed)
        profile = KinematicProfile(
            suction_multiplier=3.0 if use_suction else 0.0,
            max_velocity_mps=4.0,
        )
        sim = MatchSimulator(
            ground_truth=grid,
            start_cell=Cell(9, 0),
            initial_heading=Direction.NORTH,
            match_budget_seconds=480.0,
            reposition_time_seconds=3.0,
            enable_return_trip=enable_return_trip,
            kinematic_profile=profile,
        )
        res = sim.run_match()

        runs_data = [
            {
                "run_number": r.run_number,
                "phase": r.phase.value,
                "duration_seconds": round(r.run_time, 3),
                "is_successful": r.success,
            }
            for r in res.runs
        ]

        self._send_json(
            {
                "total_runs": res.total_successful_runs,
                "official_time": round(res.official_time, 3),
                "final_score": round(res.final_score, 2),
                "total_elapsed_seconds": round(res.total_elapsed_seconds, 2),
                "runs": runs_data,
            }
        )

    def _send_json(self, data: dict[str, Any]) -> None:
        """Serialize and send JSON response with CORS headers."""
        payload = json.dumps(data).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(payload)

    def _serve_static_file(self, filename: str, content_type: str) -> None:
        """Serve a file from the static folder."""
        filepath = os.path.join(os.path.dirname(__file__), "static", filename)
        self._serve_file(filepath, content_type)

    def _serve_file(self, filepath: str, content_type: str) -> None:
        """Serve a physical file with appropriate HTTP headers."""
        if not os.path.exists(filepath):
            self.send_error(404, "File not found")
            return

        with open(filepath, "rb") as f:
            data = f.read()

        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def start_dashboard_server(port: int = 8080) -> None:
    """Start the dashboard HTTP server."""
    with socketserver.TCPServer(("", port), DashboardHandler) as httpd:
        print(f"MMRC26 Dashboard live at http://localhost:{port}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down dashboard server.")
