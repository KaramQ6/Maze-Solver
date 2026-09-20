"""Converts Peter Harrison classic mazes and MMRC26 procedural mazes to MMS .num format."""

import os
from pathlib import Path
from maze_solver.simulator.maze_generator import generate_island_maze


def convert_txt_to_num(txt_path: Path, output_path: Path) -> bool:
    """Convert a Peter Harrison ASCII maze file (.txt) to MMS (.num) format."""
    try:
        content = txt_path.read_text(encoding="utf-8", errors="ignore")
        lines = [line.rstrip() for line in content.splitlines()]
        rows = [l for l in lines if l.startswith(("o", "|"))]

        if not rows or len(rows) < 3:
            return False

        # Height in cells is (len(rows) - 1) // 2
        height = (len(rows) - 1) // 2
        # Width in cells from first line length: (len(rows[0]) - 1) // 4
        width = (len(rows[0]) - 1) // 4

        if height <= 0 or width <= 0:
            return False

        out_lines: list[str] = []
        for y in range(height):
            for x in range(width):
                mid_r = len(rows) - 2 * y - 2
                mid_c = 2 + 4 * x

                # Bounds safety
                if mid_r - 1 < 0 or mid_r + 1 >= len(rows):
                    continue
                if mid_c - 2 < 0 or mid_c + 2 >= len(rows[mid_r]):
                    continue

                n = 1 if mid_c < len(rows[mid_r - 1]) and rows[mid_r - 1][mid_c] == "-" else 0
                s = 1 if mid_c < len(rows[mid_r + 1]) and rows[mid_r + 1][mid_c] == "-" else 0
                w = 1 if (mid_c - 2) < len(rows[mid_r]) and rows[mid_r][mid_c - 2] == "|" else 0
                e = 1 if (mid_c + 2) < len(rows[mid_r]) and rows[mid_r][mid_c + 2] == "|" else 0

                out_lines.append(f"{x} {y} {n} {e} {s} {w}")

        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text("\n".join(out_lines) + "\n", encoding="utf-8")
        return True
    except Exception as exc:
        print(f"Failed to convert {txt_path}: {exc}")
        return False


def export_mmrc26_mazes(output_dir: Path, count: int = 15) -> None:
    """Generate and export authentic MMRC26 10x10 Island-Goal mazes to MMS .num format."""
    output_dir.mkdir(parents=True, exist_ok=True)
    for seed in range(1, count + 1):
        grid = generate_island_maze(seed=seed * 101)
        out_lines: list[str] = []
        for y in range(10):
            for x in range(10):
                row = 9 - y
                col = x
                n = 1 if grid.horizontal_walls[row][col] else 0
                s = 1 if grid.horizontal_walls[row + 1][col] else 0
                w = 1 if grid.vertical_walls[row][col] else 0
                e = 1 if grid.vertical_walls[row][col + 1] else 0
                out_lines.append(f"{x} {y} {n} {e} {s} {w}")

        file_path = output_dir / f"mmrc26_10x10_seed_{seed}.num"
        file_path.write_text("\n".join(out_lines) + "\n", encoding="utf-8")


def main() -> None:
    base_dir = Path(__file__).parent
    mazes_dir = base_dir / "mazes"
    out_base = base_dir / "mazes_num"

    print("Converting Classic 16x16 Championship Mazes...")
    converted_classic = 0
    for txt_file in (mazes_dir / "classic").glob("*.txt"):
        num_file = out_base / "classic" / f"{txt_file.stem}.num"
        if convert_txt_to_num(txt_file, num_file):
            converted_classic += 1
    print(f"Successfully converted {converted_classic} classic championship mazes!")

    print("Converting Training Mazes...")
    converted_training = 0
    for txt_file in (mazes_dir / "training").glob("*.txt"):
        num_file = out_base / "training" / f"{txt_file.stem}.num"
        if convert_txt_to_num(txt_file, num_file):
            converted_training += 1
    print(f"Successfully converted {converted_training} training mazes!")

    print("Generating MMRC26 10x10 Island-Goal Competition Mazes...")
    export_mmrc26_mazes(out_base / "mmrc26_10x10", count=20)
    print("Successfully generated 20 MMRC26 10x10 Island-Goal competition mazes!")

    print(f"\nAll mazes ready in: {out_base.resolve()}")


if __name__ == "__main__":
    main()
