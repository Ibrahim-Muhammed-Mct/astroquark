import subprocess
import time


def runcommand(speed):
    subprocess.run(
        [
            "gz", "topic",
            "-t", "/astroquark/wheel_velocity",
            "-m", "gz.msgs.Double",
            "-p", f"data: {speed}",
        ],
        check=True,
    )


try:
    runcommand(1.0)
    time.sleep(5)
except KeyboardInterrupt:
    print("Stopping the rover...")
finally:
    runcommand(0.0)