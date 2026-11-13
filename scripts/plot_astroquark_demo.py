"""Render the lunar demo CSV as three synchronized, browser-ready SVG plots."""

import argparse
import csv
import math
from pathlib import Path


PANELS = (
    ("Rover speed", "m/s", ("measured_speed_m_s", "estimated_speed_m_s")),
    ("Body pitch", "deg", ("pitch_deg",)),
    ("Distance traveled", "m", ("measured_distance_m", "estimated_distance_m")),
)
COLORS = ("#00676a", "#b35214")
WIDTH, HEIGHT = 1200, 940
LEFT, RIGHT = 130, 1130
PANEL_TOPS = (203, 413, 623)
PLOT_HEIGHT = 126


def read_samples(path):
    fields = {"time_s", "x_m", "commanded_wheel_rad_s"}
    fields.update(field for _, _, series in PANELS for field in series)
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = fields.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(f"Missing CSV columns: {', '.join(sorted(missing))}")
        rows = []
        for row in reader:
            try:
                sample = {field: float(row[field]) for field in fields}
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Invalid measurement on CSV line {reader.line_num}") from exc
            if not all(map(math.isfinite, sample.values())):
                raise ValueError(f"Non-finite measurement on CSV line {reader.line_num}")
            if rows and sample["time_s"] <= rows[-1]["time_s"]:
                raise ValueError(f"Time must increase on CSV line {reader.line_num}")
            rows.append(sample)
    if len(rows) < 2:
        raise ValueError("At least two measurements are required")
    return rows


def crossing_time(samples, ramp_start_x):
    for before, after in zip(samples, samples[1:]):
        if before["x_m"] < ramp_start_x <= after["x_m"]:
            fraction = (ramp_start_x - before["x_m"]) / (after["x_m"] - before["x_m"])
            return before["time_s"] + fraction * (after["time_s"] - before["time_s"])
    return None


def nice_step(span, target_intervals):
    raw_step = span / target_intervals
    base = 10 ** math.floor(math.log10(raw_step))
    return next(base * factor for factor in (1, 2, 2.5, 5, 10) if base * factor >= raw_step)


def axis_ticks(lower, upper, step):
    return [index * step for index in range(math.ceil(lower / step), math.floor(upper / step) + 1)]


def label(value):
    return f"{value:.5g}" if value else "0"


def svg_text(x, y, content, style="", anchor="start"):
    return f'<text x="{x:.2f}" y="{y:.2f}" class="{style}" text-anchor="{anchor}">{content}</text>'


def render(samples, ramp_start_x):
    start, end = samples[0]["time_s"], samples[-1]["time_s"]
    duration = end - start
    event = crossing_time(samples, ramp_start_x)
    ramp_exit = crossing_time(samples, 4.126)
    stop_time = next((after["time_s"] for before, after in zip(samples, samples[1:])
                      if before["commanded_wheel_rad_s"] > 0 and after["commanded_wheel_rad_s"] == 0), None)

    def time_x(time):
        return LEFT + (time - start) * (RIGHT - LEFT) / duration

    parts = [f"""<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}" role="img" aria-labelledby="title desc">
  <title id="title">AstroQuark demo telemetry</title>
  <desc id="desc">Speed, pitch and distance from one Gazebo run. Speed and distance compare Gazebo position with estimates from wheel rotation.</desc>
  <style>
    text {{font-family: system-ui, 'Segoe UI', sans-serif; fill: #163243; font-size: 14px;}}
    .title {{font-size: 42px; font-weight: 720; fill: #fff;}}
    .subtitle {{fill: #c3dbe0; font-size: 17px;}}
    .metric {{font-size: 19px; font-weight: 650;}}
    .panel {{font-size: 22px; font-weight: 700;}}
    .axis {{font-size: 13px; fill: #405a65;}}
    .note {{fill: #405a65; font-size: 14px;}}
  </style>
  <rect width="{WIDTH}" height="{HEIGHT}" fill="#eff4f5"/>
  <path fill="#102a39" d="M0 0H{WIDTH}V112H0Z"/>
  <path fill="#47aaa5" d="M0 0H{WIDTH}V5H0Z"/>
  {svg_text(62, 59, 'AstroQuark telemetry', 'title')}
  {svg_text(64, 89, 'Gazebo simulation · ROS 2 measurements · Wheel kinematics', 'subtitle')}"""]
    peak_pitch = max(abs(row["pitch_deg"]) for row in samples)
    distance = samples[-1]["measured_distance_m"]
    parts.append(svg_text(65, 147, f"{duration:.1f} s recorded", "metric"))
    parts.append(svg_text(395, 147, f"{distance:.2f} m traveled", "metric"))
    parts.append(svg_text(719, 147, f"{peak_pitch:.1f}° peak absolute pitch", "metric"))

    t_ticks = axis_ticks(0, duration, nice_step(duration, 8))
    for index, ((title, units, fields), top) in enumerate(zip(PANELS, PANEL_TOPS)):
        bottom = top + PLOT_HEIGHT
        parts.append(f'<rect x="58" y="{top - 35}" width="1084" height="190" rx="11" fill="#fff"/>')
        parts.append(svg_text(77, top - 9, f"{title} ({units})", "panel"))
        values = [row[field] for row in samples for field in fields]
        low, high = min(0.0, min(values)), max(0.0, max(values))
        if high == low:
            low, high = -1.0, 1.0
        step = nice_step((high - low) * 1.05, 4)
        lower, upper = math.floor(low / step) * step, math.ceil(high / step) * step

        def value_y(value):
            return bottom - (value - lower) * PLOT_HEIGHT / (upper - lower)

        if event is not None and ramp_exit is not None:
            left = time_x(event)
            width = time_x(ramp_exit) - left
            parts.append(f'<rect x="{left:.2f}" y="{top}" width="{width:.2f}" height="{PLOT_HEIGHT}" fill="#e8f2f1"/>')
        for value in axis_ticks(lower, upper, step):
            y = value_y(value)
            parts.append(f'<path d="M{LEFT} {y:.2f}H{RIGHT}" stroke="#dce6e8"/>')
            parts.append(svg_text(LEFT - 10, y + 4, label(value), "axis", "end"))
        for relative_t in t_ticks:
            x = time_x(start + relative_t)
            parts.append(f'<path d="M{x:.2f} {top}V{bottom}" stroke="#e9eff0"/>')
            if index == len(PANELS) - 1:
                parts.append(svg_text(x, bottom + 20, label(relative_t), "axis", "middle"))
        if event is not None:
            x = time_x(event)
            parts.append(f'<path d="M{x:.2f} {top}V{bottom}" stroke="#677981" stroke-dasharray="4 4"/>')
        if stop_time is not None:
            x = time_x(stop_time)
            parts.append(f'<path d="M{x:.2f} {top}V{bottom}" stroke="#a04f43" stroke-dasharray="4 4"/>')
        if len(fields) == 1:
            parts.append(svg_text(RIGHT, top - 9, "Body IMU", "note", "end"))
        else:
            for color, name, x in zip(COLORS, ("Gazebo position", "Wheel estimate"), (775, 952)):
                parts.append(f'<path d="M{x} {top - 14}h28" stroke="{color}" stroke-width="3"/>')
                parts.append(svg_text(x + 36, top - 9, name, "note"))
        for series_index, (field, color) in enumerate(zip(fields, COLORS)):
            points = " ".join(f'{time_x(row["time_s"]):.2f},{value_y(row[field]):.2f}' for row in samples)
            dashed = ' stroke-dasharray="7 5"' if series_index else ""
            parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2.6" stroke-linejoin="round"{dashed}/>')

    parts.append(svg_text((LEFT + RIGHT) / 2, 810, "Elapsed simulation time (s)", "metric", "middle"))
    if event is not None:
        parts.append('<path d="M65 850h32" stroke="#677981" stroke-dasharray="4 4"/>')
        parts.append(svg_text(107, 855, f"Rover body reaches ramp entry at {event - start:.2f} s", "note"))
    if stop_time is not None:
        parts.append('<path d="M640 850h32" stroke="#a04f43" stroke-dasharray="4 4"/>')
        parts.append(svg_text(682, 855, f"Stop command at {stop_time - start:.2f} s", "note"))
    parts.append(svg_text(65, 882, "Wheel estimates use mean wheel rotation and wheel radius. Gazebo curves use the reference position of the rover body.", "note"))
    parts.append(svg_text(65, 905, "The estimated and measured curves can diverge on slopes because of terrain contact and slip.", "note"))
    parts.append("</svg>")
    return "\n".join(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="Lunar demo measurements")
    parser.add_argument("--ramp-start-x", type=float, default=1.65, help="Ramp entry x coordinate in meters")
    parser.add_argument("--output", type=Path, help="Output SVG path (defaults to the CSV path with .svg)")
    args = parser.parse_args()
    output = args.output or args.csv.with_suffix(".svg")
    if output.resolve() == args.csv.resolve():
        parser.error("The SVG and CSV paths must differ")
    if not math.isfinite(args.ramp_start_x):
        parser.error("--ramp-start-x must be finite")
    try:
        output.write_text(render(read_samples(args.csv), args.ramp_start_x), encoding="utf-8")
    except (OSError, ValueError) as exc:
        parser.exit(1, f"{exc}\n")
    print(output)


if __name__ == "__main__":
    main()
