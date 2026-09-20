"""Utility script to batch-register all mazes directly into Mackorone MMS.

Populates Windows Registry:
    HKCU\\Software\\mackorone\\mms\\maze-files
so all 558 mazes appear immediately in the MMS "Maze" dropdown menu without manual importing.
"""

import shutil
import winreg
from pathlib import Path


def register_all_mazes(copy_to_mms_dir: bool = True) -> int:
    repo_root = Path(__file__).resolve().parent.parent
    mazes_dir = repo_root / "mazes_num"
    mms_mazes_dir = repo_root / "tools" / "mms" / "mms" / "mazes"

    if not mazes_dir.exists():
        raise FileNotFoundError(f"Mazes directory not found at {mazes_dir}")

    # Prioritize MMRC26 10x10 mazes first, then training, then classic
    mmrc_mazes = sorted(
        (mazes_dir / "mmrc26_10x10").glob("*.num"),
        key=lambda p: int(p.stem.split("_")[-1]) if p.stem.split("_")[-1].isdigit() else 999,
    )
    training_mazes = sorted((mazes_dir / "training").glob("*.num"))
    classic_mazes = sorted((mazes_dir / "classic").glob("*.num"))

    ordered_mazes = mmrc_mazes + training_mazes + classic_mazes
    total = len(ordered_mazes)

    print(f"Found {total} total mazes:")
    print(f"  - MMRC26 10x10 Island-Goal: {len(mmrc_mazes)}")
    print(f"  - Training: {len(training_mazes)}")
    print(f"  - Classic Championships: {len(classic_mazes)}")

    if copy_to_mms_dir:
        mms_mazes_dir.mkdir(parents=True, exist_ok=True)
        for cat_dir in ["mmrc26_10x10", "training", "classic"]:
            src_cat = mazes_dir / cat_dir
            dst_cat = mms_mazes_dir / cat_dir
            if src_cat.exists():
                dst_cat.mkdir(parents=True, exist_ok=True)
                for f in src_cat.glob("*.num"):
                    shutil.copy2(f, dst_cat / f.name)
        print(f"Copied all mazes to MMS local folder: {mms_mazes_dir}")

    # Write to Windows Registry
    base_key_path = r"Software\mackorone\mms\maze-files"
    try:
        with winreg.CreateKey(winreg.HKEY_CURRENT_USER, base_key_path) as root_key:
            # Set size
            winreg.SetValueEx(root_key, "size", 0, winreg.REG_DWORD, total)

            # Register each maze
            for idx, maze_path in enumerate(ordered_mazes, start=1):
                sub_key_path = f"{base_key_path}\\{idx}"
                with winreg.CreateKey(winreg.HKEY_CURRENT_USER, sub_key_path) as sub_key:
                    # Qt paths must use forward slashes
                    posix_path = maze_path.resolve().as_posix()
                    winreg.SetValueEx(sub_key, "path", 0, winreg.REG_SZ, posix_path)

        print(f"Successfully registered {total} mazes into HKCU\\{base_key_path}!")
        return total
    except Exception as exc:
        print(f"Failed to update registry: {exc}")
        raise


if __name__ == "__main__":
    count = register_all_mazes()
    print(f"Done! {count} mazes are now available in MMS.")
