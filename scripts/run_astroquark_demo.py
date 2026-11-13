"""Run a complete lunar rover experiment from a fresh simulation."""
import argparse
import os
import secrets
import signal
import subprocess
import sys
from datetime import datetime
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
PROJECT = SCRIPTS.parent
RAMP_START_X = 1.65


def stop_process(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run without the Gazebo window")
    parser.add_argument("--output-root", type=Path, default=SCRIPTS / "results")
    parser.add_argument("--model-dir", type=Path, help="Use a model resource directory outside the demo tree")
    parser.add_argument("--speed", type=float, choices=(1.5, 1.8, 2.2), default=1.8,
                        help="Cruise wheel command in rad/s")
    args = parser.parse_args()

    description = PROJECT / "rover_description"
    simulation = description / "simulation"
    world = simulation / "astroquark_lunar_world.sdf"
    bridge = description / "config" / "astroquark_demo_bridge.yaml"
    resource_dir = args.model_dir.resolve() if args.model_dir else simulation
    if not (resource_dir / "astroquark" / "model.sdf").is_file():
        parser.error(f"Rover model not found under {resource_dir}")
    if not os.environ.get("ROS_DISTRO") == "jazzy":
        parser.error("Load ROS 2 Jazzy first with source /opt/ros/jazzy/setup.bash")

    run_id = datetime.now().strftime("astroquark_%Y%m%d_%H%M%S_%f")
    output = args.output_root.resolve() / run_id
    output.mkdir(parents=True)
    data = output / "astroquark_run.csv"
    env = os.environ.copy()
    env["GZ_PARTITION"] = run_id
    env["ASTROQUARK_DEMO_NAMESPACE"] = "/" + run_id
    env["ROS_DOMAIN_ID"] = str(40 + secrets.randbelow(60))
    env["GZ_SIM_RESOURCE_PATH"] = str(resource_dir)
    env["PYTHONUNBUFFERED"] = "1"

    print(f"Saving the experiment to {output}", flush=True)
    run_bridge = output / "bridge_config.yaml"
    bridge_lines = bridge.read_text(encoding="utf-8").splitlines()
    # An absolute ROS topic in the bridge config needs an explicit prefix.
    run_bridge.write_text(
        "\n".join(line.replace("ros_topic_name: /", f"ros_topic_name: /{run_id}/")
                  for line in bridge_lines) + "\n",
        encoding="utf-8",
    )
    simulator_cmd = ["/usr/bin/gz", "sim", "-r", "-v", "2"]
    if args.headless:
        simulator_cmd.append("-s")
    simulator_cmd.append(world.name)
    simulator_env = env.copy()
    # The ROS vendor CLI exposes transport commands only. Use system Harmonic.
    simulator_env.pop("GZ_CONFIG_PATH", None)
    simulator_env.pop("LD_LIBRARY_PATH", None)
    processes = []
    try:
        for name, command in (
            ("gazebo", simulator_cmd),
            ("bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                        "--ros-args", "-p", f"config_file:={run_bridge}"]),
        ):
            with (output / f"{name}.log").open("w") as log:
                processes.append(subprocess.Popen(
                    command, cwd=simulation, env=simulator_env if name == "gazebo" else env,
                    stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True,
                ))
        with subprocess.Popen(
            [sys.executable, str(SCRIPTS / "record_astroquark_demo.py"), str(data), "--speed", str(args.speed)],
            env=env, start_new_session=True,
        ) as recorder:
            try:
                if recorder.wait(timeout=110) != 0:
                    raise RuntimeError("Recording failed. See the terminal output and run logs.")
            except (KeyboardInterrupt, subprocess.TimeoutExpired):
                os.killpg(recorder.pid, signal.SIGINT)
                try:
                    recorder.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    stop_process(recorder)
                raise
        subprocess.run(
            [sys.executable, str(SCRIPTS / "plot_astroquark_demo.py"), str(data),
             "--ramp-start-x", str(RAMP_START_X)], check=True,
        )
        subprocess.run(
            [sys.executable, str(SCRIPTS / "verify_astroquark_demo.py"), str(data),
             "--output", str(output / "verification.json")], check=True,
        )
        if not args.headless and processes[0].poll() is None:
            print("Experiment saved. Close Gazebo or press Ctrl+C to finish.", flush=True)
            processes[0].wait()
    finally:
        for process in reversed(processes):
            stop_process(process)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
