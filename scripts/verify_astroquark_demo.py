"""Check one or more recorded lunar rover runs and summarize the evidence."""

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median


COLUMNS = (
    "time_s", "commanded_wheel_rad_s", "mean_wheel_rad_s",
    "estimated_speed_m_s", "measured_speed_m_s", "pitch_deg",
    "estimated_distance_m", "measured_distance_m", "x_m", "y_m", "z_m",
)
MAX_SAMPLE_GAP_S = 0.30
MIN_RISE_M = 0.22
PITCH_RANGE_DEG = (4.0, 14.0)
SETTLE_WINDOW_S = 0.80
MAX_STOP_SPEED_M_S = 0.02
MAX_STOP_WHEEL_RAD_S = 0.15
MAX_DISTANCE_ERROR_FRACTION = 0.12


def read_samples(path):
    with path.open(encoding="utf-8-sig", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        missing = set(COLUMNS) - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"Missing columns: {', '.join(sorted(missing))}")

        samples = []
        for row in reader:
            try:
                sample = {column: float(row[column]) for column in COLUMNS}
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Invalid numeric value on CSV line {reader.line_num}") from exc
            if not all(math.isfinite(value) for value in sample.values()):
                raise ValueError(f"Non-finite value on CSV line {reader.line_num}")
            samples.append(sample)

    if len(samples) < 2:
        raise ValueError("The CSV needs at least two samples.")
    return samples


def verify(samples, ramp_start_x, ramp_end_x, finish_x):
    first, last = samples[0], samples[-1]
    duration = last["time_s"] - first["time_s"]
    time_steps = [b["time_s"] - a["time_s"] for a, b in zip(samples, samples[1:])]
    ordered = min(time_steps) > 0.0
    max_gap = max(time_steps)

    approach = [row for row in samples if row["x_m"] < ramp_start_x - 0.5]
    ramp = [row for row in samples if ramp_start_x - 0.5 <= row["x_m"] <= ramp_end_x + 0.5]
    reached_ramp_start = next((row for row in samples if row["x_m"] >= ramp_start_x), None)
    reached_ramp_end = next((row for row in samples if row["x_m"] >= ramp_end_x), None)
    reached_goal = next((row for row in samples if row["x_m"] >= finish_x), None)

    settled = [row for row in samples if last["time_s"] - row["time_s"] <= SETTLE_WINDOW_S + 1e-6]
    settle_coverage = last["time_s"] - settled[0]["time_s"]
    stop_speed = max(abs(row["measured_speed_m_s"]) for row in settled)
    stop_wheel_rate = max(abs(row["mean_wheel_rad_s"]) for row in settled)
    stop_commands = all(abs(row["commanded_wheel_rad_s"]) <= 1e-6 for row in settled)

    rise = median(row["z_m"] for row in settled) - median(row["z_m"] for row in approach) if approach else None
    peak_pitch = max((row["pitch_deg"] for row in ramp), default=None)
    measured_distance = last["measured_distance_m"]
    estimated_distance = last["estimated_distance_m"]
    distance_gap = estimated_distance - measured_distance
    relative_gap = abs(distance_gap) / measured_distance if measured_distance > 0.0 else None

    checks = {
        "sampling": ordered and first["time_s"] >= 0 and max_gap <= MAX_SAMPLE_GAP_S + 1e-6,
        "ramp_crossing": (
            bool(approach) and reached_ramp_start is not None and reached_ramp_end is not None
            and reached_ramp_start["time_s"] < reached_ramp_end["time_s"]
            and rise is not None and rise >= MIN_RISE_M
        ),
        "nose_up_pitch": peak_pitch is not None and PITCH_RANGE_DEG[0] <= peak_pitch <= PITCH_RANGE_DEG[1],
        "goal_reached": reached_goal is not None and last["x_m"] >= finish_x,
        "settled_stop": (
            ordered and settle_coverage >= SETTLE_WINDOW_S - max_gap - 1e-6
            and settle_coverage >= SETTLE_WINDOW_S / 2 and stop_commands
            and stop_speed <= MAX_STOP_SPEED_M_S and stop_wheel_rate <= MAX_STOP_WHEEL_RAD_S
        ),
        "distance_agreement": (
            measured_distance >= finish_x - first["x_m"] - 0.05
            and relative_gap is not None and relative_gap <= MAX_DISTANCE_ERROR_FRACTION
        ),
    }
    metrics = {
        "sample_count": len(samples),
        "duration_s": duration,
        "max_sample_gap_s": max_gap,
        "ramp_start_s": reached_ramp_start["time_s"] if reached_ramp_start is not None else None,
        "ramp_end_s": reached_ramp_end["time_s"] if reached_ramp_end is not None else None,
        "goal_s": reached_goal["time_s"] if reached_goal is not None else None,
        "final_x_m": last["x_m"],
        "rise_m": rise,
        "peak_ramp_pitch_deg": peak_pitch,
        "peak_measured_speed_m_s": max(row["measured_speed_m_s"] for row in samples),
        "max_settled_speed_m_s": stop_speed,
        "max_settled_wheel_rad_s": stop_wheel_rate,
        "measured_distance_m": measured_distance,
        "estimated_distance_m": estimated_distance,
        "distance_gap_m": distance_gap,
        "absolute_distance_error_pct": 100.0 * relative_gap if relative_gap is not None else None,
    }
    return {"passed": all(checks.values()), "checks": checks, "metrics": metrics}


def compare_runs(runs):
    complete = [run["metrics"] for run in runs if run["passed"]]
    comparison = {"passing_runs": len(complete), "total_runs": len(runs)}
    if len(complete) >= 2:
        for field in ("duration_s", "measured_distance_m", "peak_ramp_pitch_deg", "final_x_m"):
            values = [metrics[field] for metrics in complete]
            comparison[f"{field}_range"] = [min(values), max(values)]
    return comparison


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path, help="Recorded lunar run CSV files")
    parser.add_argument("--output", type=Path, help="Write the JSON report to this path")
    parser.add_argument("--ramp-start-x", type=float, default=1.65)
    parser.add_argument("--ramp-end-x", type=float, default=4.126)
    parser.add_argument("--finish-x", type=float, default=5.25)
    args = parser.parse_args()
    coordinates = (args.ramp_start_x, args.ramp_end_x, args.finish_x)
    if not all(math.isfinite(value) for value in coordinates) or not coordinates[0] < coordinates[1] < coordinates[2]:
        parser.error("Expected finite coordinates with ramp start < ramp end < finish.")
    if args.output and any(args.output.resolve() == run.resolve() for run in args.runs):
        parser.error("The JSON output must differ from each input CSV.")

    runs = []
    for path in args.runs:
        try:
            result = verify(read_samples(path), *coordinates)
        except (OSError, ValueError) as exc:
            result = {"passed": False, "error": str(exc)}
        runs.append({"csv": str(path), **result})

    report = {
        "passed": all(run["passed"] for run in runs),
        "criteria": {
            "ramp_start_x_m": args.ramp_start_x,
            "ramp_end_x_m": args.ramp_end_x,
            "finish_x_m": args.finish_x,
            "max_sample_gap_s": MAX_SAMPLE_GAP_S,
            "minimum_rise_m": MIN_RISE_M,
            "ramp_pitch_range_deg": PITCH_RANGE_DEG,
            "settle_window_s": SETTLE_WINDOW_S,
            "max_stop_speed_m_s": MAX_STOP_SPEED_M_S,
            "max_stop_wheel_rad_s": MAX_STOP_WHEEL_RAD_S,
            "max_distance_error_pct": 100 * MAX_DISTANCE_ERROR_FRACTION,
        },
        "runs": runs,
        "comparison": compare_runs(runs),
    }
    rendered = json.dumps(report, indent=2, allow_nan=False)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
        for run in runs:
            failed_checks = [check for check, passed in run.get("checks", {}).items() if not passed]
            reason = run.get("error") or ", ".join(failed_checks)
            status = f"FAIL ({reason})" if reason else "PASS"
            print(f"{status}: {run['csv']}")
        print(f"Verification report: {args.output}")
    else:
        print(rendered)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
