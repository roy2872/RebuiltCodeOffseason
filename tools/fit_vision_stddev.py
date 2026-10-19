"""Fits the vision std dev model (VisionFilter) from stationary test logs.

The model, for a still robot:   sigma = LinearBase * (1 + kDistance * d^2) / sqrt(tagCount)
(times the camera factor, and Megatag2Factor for MegaTag2 poses).

Procedure:
  1. Put the robot still at 6-8 spots: near and far, seeing 1 tag and several tags. Enable it (so
     MegaTag2 is used, if that's what you want to calibrate) and log ~10 s at each spot.
  2. In AdvantageScope, for each spot select its time range and export a CSV with the fields
     Vision/Camera0/Calibration/{Timestamp,X,Y,Distance,TagCount}. One CSV per spot.
  3. python tools/fit_vision_stddev.py spot1.csv spot2.csv ... [--megatag2-factor 0.5] [--camera-factor 1.0]
  4. Put the printed LinearBase and kDistance into the Vision/StdDev TunableNumbers (or the defaults
     in VisionFilter.java).
"""

import argparse
import csv
import math
import statistics


def find_column(header, suffix):
    for i, name in enumerate(header):
        if name.endswith(suffix):
            return i
    raise SystemExit(f"no column ending in '{suffix}' in {header}")


def load_spot(path):
    """Unique observations (by capture timestamp) from one spot's CSV."""
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        cols = {key: find_column(header, "Calibration/" + key) for key in ("Timestamp", "X", "Y", "Distance", "TagCount")}
        seen = {}
        for row in reader:
            try:
                values = {key: float(row[i]) for key, i in cols.items()}
            except (ValueError, IndexError):
                continue  # blank cells before the first observation
            seen[values["Timestamp"]] = values
    return list(seen.values())


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("spots", nargs="+", help="one CSV per stationary spot")
    parser.add_argument("--megatag2-factor", type=float, default=1.0,
                        help="Vision/StdDev/Megatag2Factor if the spots are MegaTag2 poses (default 0.5 in code)")
    parser.add_argument("--camera-factor", type=float, default=1.0, help="the camera's cameraStdDevFactors entry")
    args = parser.parse_args()

    points = []  # (d^2, sigma * sqrt(n) / factors)
    print(f"{'spot':30s} {'samples':>7s} {'dist m':>7s} {'tags':>5s} {'sigma m':>8s}")
    for path in args.spots:
        obs = load_spot(path)
        if len(obs) < 5:
            print(f"{path:30s} only {len(obs)} samples, skipped")
            continue
        sx = statistics.stdev(o["X"] for o in obs)
        sy = statistics.stdev(o["Y"] for o in obs)
        sigma = math.sqrt((sx * sx + sy * sy) / 2)
        d = statistics.mean(o["Distance"] for o in obs)
        n = statistics.mean(o["TagCount"] for o in obs)
        print(f"{path:30s} {len(obs):7d} {d:7.2f} {n:5.1f} {sigma:8.4f}")
        points.append((d * d, sigma * math.sqrt(n) / (args.megatag2_factor * args.camera_factor)))

    if len(points) < 2:
        raise SystemExit("need at least 2 usable spots")
    # Least squares for y = a0 + a1 * d^2, then LinearBase = a0 and kDistance = a1 / a0
    xs, ys = zip(*points)
    mx, my = statistics.mean(xs), statistics.mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    a1 = sum((x - mx) * (y - my) for x, y in points) / sxx if sxx > 0 else 0.0
    a0 = my - a1 * mx
    if a0 <= 0:
        print("warning: fit gave a non-positive base; spots are too noisy or too few. Using the mean.")
        a0, a1 = my, 0.0
    print()
    print(f"LinearBase = {a0:.5f}")
    print(f"kDistance  = {a1 / a0:.4f}")
    print("Tip: the static spread is a lower bound; moving robots get the motion and latency factors on top.")


if __name__ == "__main__":
    main()
