"""Generates src/main/deploy/pathplanner/navgrid.json for the 2026 REBUILT field.

PathPlanner's on-the-fly pathfinding (AD*) avoids the cells marked true here. Geometry is taken from
maple-sim's Arena2026Rebuilt (the same field the simulator uses).

This robot can't fit in the trench and only crosses the bumps with a dedicated straight, slow,
diagonal crossing (frc.robot.auto.BumpCrossing). So the whole hub line (hub, bumps and trenches) is
a wall for the pathfinder, and paths never try to cross it on their own.

Obstacles and walls are grown by the radius the robot sweeps when it turns with the intake fully
deployed, so a path is safe at any heading.

Run from the repo root:  python tools/generate_navgrid.py
"""

import json
import math
from pathlib import Path

INCH = 0.0254
FIELD_LENGTH = 16.541
FIELD_WIDTH = 8.052
NODE_SIZE = 0.3

# Keep in sync with DriveConstants (BUMPER_LENGTH_X, BUMPER_WIDTH_Y, INTAKE_MAX_EXTENSION)
BUMPER_LENGTH_X = 0.89
BUMPER_WIDTH_Y = 0.89
INTAKE_MAX_EXTENSION = 0.26
SWEPT_RADIUS = math.hypot(BUMPER_LENGTH_X / 2 + INTAKE_MAX_EXTENSION, BUMPER_WIDTH_Y / 2)

# Blue-side obstacles as (center x, center y, size x, size y), meters
HUB_X = 4.5974
HUB_LINE_DEPTH = 53 * INCH  # the trench structure is the deepest part of the line
BLUE_OBSTACLES = [
    (HUB_X, FIELD_WIDTH / 2, HUB_LINE_DEPTH, FIELD_WIDTH),  # hub, bumps and trenches
    (42 * INCH, 159 * INCH, 2 * INCH, 47 * INCH),  # tower poles
]


def mirrored(obstacle):
    """The red-side copy: the field is rotationally symmetric about its center."""
    x, y, sx, sy = obstacle
    return (FIELD_LENGTH - x, FIELD_WIDTH - y, sx, sy)


def distance_to_rect(px, py, obstacle):
    x, y, sx, sy = obstacle
    dx = max(abs(px - x) - sx / 2, 0.0)
    dy = max(abs(py - y) - sy / 2, 0.0)
    return math.hypot(dx, dy)


def main():
    obstacles = BLUE_OBSTACLES + [mirrored(o) for o in BLUE_OBSTACLES]
    cols = math.ceil(FIELD_LENGTH / NODE_SIZE)
    rows = math.ceil(FIELD_WIDTH / NODE_SIZE)
    grid = []
    for r in range(rows):
        row = []
        for c in range(cols):
            px = (c + 0.5) * NODE_SIZE
            py = (r + 0.5) * NODE_SIZE
            near_wall = min(px, py, FIELD_LENGTH - px, FIELD_WIDTH - py) < SWEPT_RADIUS
            blocked = near_wall or any(distance_to_rect(px, py, o) < SWEPT_RADIUS for o in obstacles)
            row.append(blocked)
        grid.append(row)

    navgrid = {
        "field_size": {"x": FIELD_LENGTH, "y": FIELD_WIDTH},
        "nodeSizeMeters": NODE_SIZE,
        "grid": grid,
    }
    out = Path(__file__).resolve().parent.parent / "src/main/deploy/pathplanner/navgrid.json"
    out.write_text(json.dumps(navgrid))

    for row in reversed(grid):
        print("".join("#" if cell else "." for cell in row))
    print(f"swept radius {SWEPT_RADIUS:.3f} m; wrote {out} ({cols}x{rows})")


if __name__ == "__main__":
    main()
