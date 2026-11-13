import csv
import math
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


def read_rover_position():
    result = subprocess.run(
        ["gz", "topic", "-e", "-t", "/world/astroquark_world/pose/info", "-n", "1"],
        capture_output=True,
        text=True,
        check=True,
        timeout=5,
    )

    for pose in re.findall(r"^pose \{(.*?)^\}", result.stdout, re.M | re.S):
        if 'name: "astroquark"' not in pose:
            continue

        position = re.search(r"\bposition\s*\{([^}]*)\}", pose)
        if position is None:
            raise RuntimeError("Position is missing for astroquark.")

        coords = dict(re.findall(r"([xyz]):\s*(\S+)", position.group(1)))
        return (
            float(coords.get("x", 0)),
            float(coords.get("y", 0)),
            float(coords.get("z", 0)),
        )

    raise RuntimeError("Could not find astroquark in the Gazebo snapshot.")


def measure_displacement():
    x_start, y_start, _ = read_rover_position()

    drive_script = Path(__file__).resolve().with_name("drive_astroquark.py")
    subprocess.run([sys.executable, str(drive_script)], check=True)

    time.sleep(1)  # Give the rover time to settle after the stop command.
    x_end, y_end, _ = read_rover_position()

    return math.hypot(x_end - x_start, y_end - y_start)


def save_result(displacement):
    results_file = Path(__file__).resolve().parent / "results" / "astroquark_motion_results.csv"
    results_file.parent.mkdir(parents=True, exist_ok=True)
    needs_header = not results_file.exists() or results_file.stat().st_size == 0

    with results_file.open("a", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        if needs_header:
            writer.writerow(["timestamp", "horizontal_displacement_m"])

        writer.writerow([
            datetime.now().astimezone().isoformat(timespec="seconds"),
            f"{displacement:.6f}",
        ])

    return results_file


if __name__ == "__main__":
    displacement = measure_displacement()
    results_file = save_result(displacement)

    print(f"Horizontal displacement: {displacement:.6f} m")
    print(f"Results saved to: {results_file}")