"""Drive the simulated rover straight and record its motion in ROS 2."""
import argparse
import csv
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

WHEEL_RADIUS_M = 0.131935968995
WHEEL_NAMES = tuple(
    f"{side}_{position}_wheel_joint"
    for side in ("left", "right")
    for position in ("front", "mid", "rear")
)
MAX_SAMPLE_AGE_S = 0.25
RAMP_START_X_M = 1.65
RAMP_END_X_M = 4.126
APPROACH_LENGTH_M = 0.35
RAMP_SPEED_FACTOR = 0.80
FINAL_SPEED_FACTOR = 0.45
COLUMNS = (
    "time_s", "commanded_wheel_rad_s", "mean_wheel_rad_s",
    "estimated_speed_m_s", "measured_speed_m_s", "pitch_deg",
    "estimated_distance_m", "measured_distance_m", "x_m", "y_m", "z_m",
)


class LunarRecorder(Node):
    def __init__(self):
        super().__init__("astroquark_demo_recorder", namespace=os.environ.get("ASTROQUARK_DEMO_NAMESPACE", "/"))
        self.clock_s = None
        self.pose = None
        self.wheels = None
        self.imu = None
        self.command = self.create_publisher(Float64, "astroquark/wheel_velocity", 10)
        self.create_subscription(Clock, "clock", self.receive_clock, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "astroquark/reference_pose", self.receive_pose, qos_profile_sensor_data)
        self.create_subscription(JointState, "astroquark/joint_states", self.receive_joints, qos_profile_sensor_data)
        self.create_subscription(Imu, "astroquark/imu", self.receive_imu, qos_profile_sensor_data)

    def receive_clock(self, message):
        self.clock_s = message.clock.sec + message.clock.nanosec * 1e-9

    def receive_pose(self, message):
        for transform in message.transforms:
            if transform.child_frame_id == "astroquark":
                p = transform.transform.translation
                self.pose = (self.clock_s, (p.x, p.y, p.z))
                break

    def receive_joints(self, message):
        velocities = dict(zip(message.name, message.velocity))
        if all(name in velocities for name in WHEEL_NAMES):
            self.wheels = (self.clock_s, sum(abs(velocities[name]) for name in WHEEL_NAMES) / len(WHEEL_NAMES))

    def receive_imu(self, message):
        q = message.orientation
        # ROS body axes are x-forward, y-left, z-up. Report nose-up as positive.
        sine_pitch = 2.0 * (q.w * q.y - q.z * q.x)
        pitch_deg = -math.degrees(math.asin(max(-1.0, min(1.0, sine_pitch))))
        self.imu = (self.clock_s, pitch_deg)

    def publish_speed(self, speed):
        self.command.publish(Float64(data=float(speed)))

    def ready(self):
        if self.clock_s is None or self.command.get_subscription_count() == 0:
            return False
        return all(
            sample is not None and sample[0] is not None
            and 0 <= self.clock_s - sample[0] <= MAX_SAMPLE_AGE_S
            for sample in (self.pose, self.wheels, self.imu)
        )

    def record(self, output, speed, finish_x):
        wait_until = time.monotonic() + 25
        while not self.ready():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() >= wait_until:
                missing = [name for name in ("clock_s", "pose", "wheels", "imu") if getattr(self, name) is None]
                raise RuntimeError(f"Fresh simulation data or bridge unavailable. Missing: {missing}")

        settle_until = self.clock_s + 1.0
        last_progress = time.monotonic()
        previous_clock = self.clock_s
        start_s = None
        last_sample_s = None
        previous_pose = None
        previous_estimated_speed = 0.0
        estimated_distance = measured_distance = 0.0
        stop_s = None
        reached_goal = False
        samples = 0

        with output.open("x", encoding="utf-8-sig", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(COLUMNS)
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                now = self.clock_s
                if now < previous_clock:
                    raise RuntimeError("The simulation clock reset during the run.")
                if now > previous_clock:
                    previous_clock = now
                    last_progress = time.monotonic()
                if time.monotonic() - last_progress > 5:
                    raise RuntimeError("The simulation clock stopped advancing.")
                if now < settle_until or (last_sample_s is not None and now - last_sample_s < 0.099999):
                    continue
                if not self.ready():
                    raise RuntimeError("A simulation stream became stale during the run.")

                position = self.pose[1]
                wheel_rate = self.wheels[1]
                estimated_speed = WHEEL_RADIUS_M * wheel_rate
                measured_speed = 0.0
                if start_s is None:
                    start_s = now
                else:
                    dt = now - last_sample_s
                    step_distance = math.dist(position, previous_pose)
                    measured_speed = step_distance / dt
                    measured_distance += step_distance
                    estimated_distance += (previous_estimated_speed + estimated_speed) * dt / 2.0

                if stop_s is None and (position[0] >= finish_x or now - start_s >= 55.0):
                    reached_goal = position[0] >= finish_x
                    stop_s = now
                command = 0.0
                if stop_s is None:
                    # The predefined straight route uses the reference position
                    # to stage the showcase, including a gentler final approach.
                    if position[0] >= finish_x - APPROACH_LENGTH_M:
                        command = speed * FINAL_SPEED_FACTOR
                    elif RAMP_START_X_M - APPROACH_LENGTH_M <= position[0] <= RAMP_END_X_M + APPROACH_LENGTH_M:
                        command = speed * RAMP_SPEED_FACTOR
                    else:
                        command = speed
                self.publish_speed(command)
                writer.writerow(f"{value:.8f}" for value in (
                    now - start_s, command, wheel_rate, estimated_speed, measured_speed,
                    self.imu[1], estimated_distance, measured_distance, *position,
                ))
                samples += 1
                last_sample_s = now
                previous_pose = position
                previous_estimated_speed = estimated_speed
                if stop_s is not None and now - stop_s >= 1.5:
                    break

        if not reached_goal:
            raise RuntimeError(f"Rover did not reach x={finish_x:.2f} m. Partial data: {output}")
        print(f"Recorded {samples} samples. Final x={position[0]:.3f} m. CSV: {output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--speed", type=float, default=1.8, help="Wheel command in rad/s")
    parser.add_argument("--finish-x", type=float, default=5.25, help="Stop at this world x coordinate in m")
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not 0 < args.speed <= 2.5:
        parser.error("--speed must be greater than 0 and at most 2.5 rad/s")
    if not math.isfinite(args.finish_x) or not RAMP_END_X_M + APPROACH_LENGTH_M < args.finish_x <= 6.0:
        parser.error("--finish-x must be after the ramp and at most 6.0 m")

    rclpy.init()
    node = LunarRecorder()
    try:
        node.record(args.output, args.speed, args.finish_x)
    finally:
        if rclpy.ok():
            for _ in range(3):
                node.publish_speed(0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
