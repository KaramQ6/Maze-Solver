"""A real stdin/stdout algorithm used by the native MMS regression checks."""

import os
import sys
import time


def report(message: str) -> None:
    sys.stderr.write(message + "\n")
    sys.stderr.flush()


def command(request: str) -> str:
    sys.stdout.write(request + "\n")
    sys.stdout.flush()
    response = sys.stdin.readline().strip()
    if not response:
        raise RuntimeError(f"MMS closed while waiting for {request}")
    return response


def movement(request: str, label: str) -> None:
    started = time.perf_counter()
    response = command(request)
    if response != "ack":
        raise RuntimeError(f"{request}: expected ack, got {response}")
    report(f"{label} {time.perf_counter() - started:.6f}")


def main() -> None:
    case = os.environ["MMS_PROBE_CASE"]
    report(
        f"START {os.environ['MMS_START_X']} {os.environ['MMS_START_Y']} "
        f"{os.environ['MMS_START_HEADING']}"
    )
    report("BEFORE_MOVE")
    movement("moveForward", "MOVE1")
    if case == "snapshot":
        movement("moveForward", "MOVE2")
    else:
        movement("turnLeft", "TURN")
        sys.stdout.write("setMotionMode speed\n")
        sys.stdout.flush()
        movement("moveForward", "MOVE2")
    if case == "reset":
        report("RESET " + command("wasReset"))
    if command("ackReset") != "ack":
        raise RuntimeError("Reset was not acknowledged")
    walls = [command("wall" + side) for side in ("Front", "Right", "Left", "Back")]
    report("WALLS " + ",".join(walls))
    report("DONE")


if __name__ == "__main__":
    main()
