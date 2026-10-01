"""Build the customized MMS using a project-local Windows Qt/MinGW SDK."""

import argparse
import contextlib
import logging
import os
import shutil
import subprocess
import sys
from collections.abc import Iterator
from pathlib import Path

UPSTREAM = "https://github.com/mackorone/mms.git"
REVISION = "4ed1df48c5223b6b8b422d6e6cf2569ef47d8d8c"
QT_VERSION = "6.8.3"
LOGGER = logging.getLogger(__name__)


def run(args: list[str], cwd: Path, env: dict[str, str]) -> None:
    subprocess.run(args, cwd=cwd, env=env, check=True)


@contextlib.contextmanager
def ascii_workspace(root: Path) -> Iterator[Path]:
    """Map only this workspace; always release the temporary drive."""
    for letter in "ZYXWVUTSRQP":
        drive = f"{letter}:"
        if not Path(drive + "\\").exists():
            subprocess.run(["subst", drive, str(root)], check=True)
            try:
                yield Path(drive + "\\")
            finally:
                subprocess.run(["subst", drive, "/D"], check=True)
            return
    raise RuntimeError("No free drive letter for the Unicode-safe build path")


def prepare_sdk(base: Path, env: dict[str, str], setup: bool) -> None:
    qt = base / "sdk" / QT_VERSION / "mingw_64"
    compiler = base / "sdk" / "Tools" / "mingw1310_64" / "bin"
    deps = base / "build-deps"
    if setup and not (deps / "aqt").exists():
        run(
            [sys.executable, "-m", "pip", "install", "--target", str(deps), "aqtinstall==3.3.0"],
            base,
            env,
        )
    env["PYTHONPATH"] = str(deps)
    if not (qt / "bin" / "qmake.exe").exists():
        if not setup:
            raise RuntimeError("Qt SDK missing; run with --setup first")
        # Aqt's post-extraction patcher assumes UTF-8 qmake output on Windows.
        # The extracted SDK is relocatable; qt.conf below supplies its prefix.
        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "aqt",
                "install-qt",
                "windows",
                "desktop",
                QT_VERSION,
                "win64_mingw",
                "--archives",
                "qtbase",
                "--outputdir",
                str(base / "sdk"),
            ],
            cwd=base,
            env=env,
        )
        if result.returncode and not (qt / "bin" / "qmake.exe").exists():
            raise RuntimeError("Qt download failed; see aqt output above")
    required_qt_files = (
        "bin/qmake.exe",
        "bin/windeployqt.exe",
        "bin/Qt6Core.dll",
        "include/QtCore/qglobal.h",
        "lib/libQt6Core.a",
    )
    if any(not (qt / name).is_file() for name in required_qt_files):
        raise RuntimeError("Qt SDK extraction is incomplete; see aqt output above")
    if not (compiler / "g++.exe").exists():
        if not setup:
            raise RuntimeError("MinGW SDK missing; run with --setup first")
        run(
            [
                sys.executable,
                "-m",
                "aqt",
                "install-tool",
                "windows",
                "desktop",
                "tools_mingw1310",
                "qt.tools.win64_mingw1310",
                "--outputdir",
                str(base / "sdk"),
            ],
            base,
            env,
        )
    (qt / "bin" / "qt.conf").write_text(f"[Paths]\nPrefix={qt.as_posix()}\n", encoding="utf-8")
    env["PATH"] = os.pathsep.join([str(compiler), str(qt / "bin"), env["PATH"]])
    toolchain = compiler.parent.as_posix()
    gcc_lib = f"{toolchain}/lib/gcc/x86_64-w64-mingw32/13.1.0"
    env["GCC_EXEC_PREFIX"] = f"{toolchain}/lib/gcc/"
    env["COMPILER_PATH"] = os.pathsep.join([compiler.as_posix(), gcc_lib])
    env["LIBRARY_PATH"] = os.pathsep.join(
        [f"{toolchain}/x86_64-w64-mingw32/lib", f"{toolchain}/lib", gcc_lib]
    )


def prepare_source(base: Path, env: dict[str, str]) -> None:
    source = base / "source"
    if not source.exists():
        run(
            ["git", "clone", "--filter=blob:none", "--no-checkout", UPSTREAM, str(source)],
            base,
            env,
        )
        run(["git", "sparse-checkout", "set", "src"], source, env)
        run(["git", "checkout", "--detach", REVISION], source, env)
    revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=source, env=env)
    if revision.decode("ascii").strip() != REVISION:
        raise RuntimeError(f"MMS source must be at {REVISION}; source was not reset")
    patch = base / "patches" / "mmrc26.patch"
    applied = subprocess.run(
        ["git", "apply", "--reverse", "--check", str(patch)],
        cwd=source,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if applied.returncode:
        run(["git", "apply", "--check", str(patch)], source, env)
        run(["git", "apply", str(patch)], source, env)
    for extension in (base / "extensions").iterdir():
        if extension.suffix in {".h", ".cpp"}:
            shutil.copy2(extension, source / "src" / extension.name)


def build(base: Path, env: dict[str, str], test: bool) -> None:
    qt_bin = base / "sdk" / QT_VERSION / "mingw_64" / "bin"
    build_dir = base / "build"
    build_dir.mkdir(exist_ok=True)
    run(
        [
            str(qt_bin / "qmake.exe"),
            str(base / "source" / "src" / "mms.pro"),
            "-after",
            "CONFIG+=release",
            "CONFIG-=debug",
            "CONFIG-=debug_and_release",
            "OBJECTS_DIR=release-obj",
            "MOC_DIR=release-moc",
            "RCC_DIR=release-rcc",
            f"DESTDIR={(base / 'bin').as_posix()}",
        ],
        build_dir,
        env,
    )
    run(["mingw32-make", "-s", "-j", str(min(os.cpu_count() or 2, 8))], build_dir, env)
    run(
        [
            str(qt_bin / "windeployqt.exe"),
            "--release",
            "--no-translations",
            "--no-system-d3d-compiler",
            str(base / "bin" / "mms.exe"),
        ],
        build_dir,
        env,
    )
    shutil.copy2(base / "source" / "LICENSE", base / "bin" / "LICENSE.mms.txt")
    if test:
        test_dir = build_dir / "tests"
        test_dir.mkdir(exist_ok=True)
        run([str(qt_bin / "qmake.exe"), str(base / "tests" / "native.pro")], test_dir, env)
        run(["mingw32-make", "-s", "-j", str(min(os.cpu_count() or 2, 8))], test_dir, env)
        env["MMS_TEST_ROOT"] = str(base.parents[1])
        env["MMS_TEST_PYTHON"] = sys.executable
        report = test_dir / "native-results.txt"
        run([str(test_dir / "release" / "native_tests.exe"), "-o", f"{report},txt"], test_dir, env)
        LOGGER.info("%s", report.read_text(encoding="utf-8"))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup", action="store_true", help="Download the local build SDK")
    parser.add_argument("--test", action="store_true", help="Run native UI/protocol regressions")
    options = parser.parse_args()
    if sys.platform != "win32":
        parser.error("This deployment script targets Windows")
    root = Path(__file__).resolve().parents[2]
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        with ascii_workspace(root) as workspace:
            base = workspace / "tools" / "mms"
            env = dict(os.environ, PYTHONUTF8="1")
            prepare_sdk(base, env, options.setup)
            prepare_source(base, env)
            build(base, env, options.test)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        LOGGER.error("MMS build failed: %s", error)
        sys.exit(1)
    LOGGER.info("Built tools/mms/bin/mms.exe. Launch with launch_mms.bat.")


if __name__ == "__main__":
    main()
