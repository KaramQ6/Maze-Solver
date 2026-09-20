"""CLI entrypoint for running the interactive visual dashboard."""

import argparse
import webbrowser

from maze_solver.dashboard.server import start_dashboard_server


def main() -> None:
    """Parse CLI arguments and launch the dashboard server."""
    parser = argparse.ArgumentParser(description="MMRC26 Micromouse Interactive Web Dashboard")
    parser.add_argument("--port", type=int, default=8080, help="Port to bind server")
    parser.add_argument(
        "--no-browser", action="store_true", help="Do not open browser automatically"
    )
    args = parser.parse_args()

    url = f"http://localhost:{args.port}"
    if not args.no_browser:
        webbrowser.open(url)

    start_dashboard_server(port=args.port)


if __name__ == "__main__":
    main()
