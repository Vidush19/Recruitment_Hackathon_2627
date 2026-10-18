#!/usr/bin/env python3
"""Racing line and speed profile, generated from the occupancy grid.

    ros2 run team_driver make_raceline                       # writes maps/icra26_compete_raceline.csv
    ros2 run team_driver make_raceline --margin 0.30 --plot /tmp/line.png

Two separate jobs live here.

1. **Geometry, offline.** Start from the traced centreline, measure how far the
   track extends either side of it along the normal, and move every point
   sideways to minimise the curvature of the closed loop - the classic
   minimum-curvature line (Heilmeier et al., 2019). That is what produces "wide
   in, apex, wide out": the straightest path the corridor allows. It is solved
   as a box-constrained least-squares problem in the lateral offsets, and
   re-linearised a few times because the normals move with the line.

2. **Speed, at start-up.** `speed_profile()` turns curvature into a target speed
   for every point: a lateral-grip cap per point, then forward and backward
   passes so the car reaches each corner already at its speed. The driver calls
   it with its own parameters, so the grip and braking limits can be tuned
   without regenerating the line.

Only numpy: the image ships it, and nothing here needs more.
"""

import argparse
import math
import os

import numpy as np

FREE_VALUE = 254          # track surface in the traced grid; 0 = duct, 205 = outfield


# ----------------------------------------------------------------------------
# Map and line I/O
# ----------------------------------------------------------------------------

def read_pgm(path):
    with open(path, 'rb') as handle:
        data = handle.read()
    fields, pos = [], 2
    while len(fields) < 3:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while data[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        fields.append(int(data[start:pos]))
    width, height, _ = fields
    pixels = np.frombuffer(data[pos + 1:pos + 1 + width * height], dtype=np.uint8)
    return pixels.reshape(height, width)


class Grid:
    """The occupancy grid, with world <-> cell conversion. Row 0 is the top (max y)."""

    def __init__(self, yaml_path):
        import yaml                                     # noqa: PLC0415 - in the image
        with open(yaml_path) as handle:
            meta = yaml.safe_load(handle)
        image = os.path.join(os.path.dirname(yaml_path), meta['image'])
        self.pixels = read_pgm(image)
        self.free = self.pixels == FREE_VALUE
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.height, self.width = self.free.shape
        rows, cols = np.nonzero(~self.free)
        self.blocked_xy = np.column_stack(self.to_world(rows, cols))

    def to_cell(self, x, y):
        col = np.floor((np.asarray(x) - self.ox) / self.res).astype(int)
        row = self.height - 1 - np.floor((np.asarray(y) - self.oy) / self.res).astype(int)
        return row, col

    def to_world(self, row, col):
        x = self.ox + (np.asarray(col) + 0.5) * self.res
        y = self.oy + (self.height - 1 - np.asarray(row) + 0.5) * self.res
        return x, y

    def is_free(self, x, y):
        row, col = self.to_cell(x, y)
        inside = (row >= 0) & (row < self.height) & (col >= 0) & (col < self.width)
        out = np.zeros(np.shape(row), dtype=bool)
        out[inside] = self.free[row[inside], col[inside]]
        return out

    def clearance(self, points):
        """Distance from each point to the nearest non-track cell centre."""
        out = np.empty(len(points))
        for i, p in enumerate(points):
            out[i] = np.min(np.hypot(*(self.blocked_xy - p).T))
        return out


def load_xy_csv(path):
    rows = []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = [p for p in line.replace(';', ',').split(',') if p.strip()]
            rows.append([float(p) for p in parts])
    return np.asarray(rows)


def load_raceline(path):
    """Read a `s; x; y; psi; kappa; vx; ax` CSV, or a bare `x, y` one. Returns (N, 2) xy."""
    data = load_xy_csv(path)
    return data[:, 1:3] if data.shape[1] >= 7 else data[:, 0:2]


# ----------------------------------------------------------------------------
# Closed-loop geometry
# ----------------------------------------------------------------------------

def resample_closed(points, spacing):
    loop = np.vstack([points, points[:1]])
    seg = np.hypot(*np.diff(loop, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    count = max(8, int(round(s[-1] / spacing)))
    targets = np.linspace(0.0, s[-1], count, endpoint=False)
    return np.column_stack([np.interp(targets, s, loop[:, 0]),
                            np.interp(targets, s, loop[:, 1])])


def smooth_closed(points, passes=1):
    for _ in range(passes):
        points = 0.25 * np.roll(points, 1, 0) + 0.5 * points + 0.25 * np.roll(points, -1, 0)
    return points


def heading_and_curvature(points):
    """Heading and signed curvature (left positive) of a closed, evenly spaced loop."""
    d1 = (np.roll(points, -1, 0) - np.roll(points, 1, 0)) / 2.0
    d2 = np.roll(points, -1, 0) - 2.0 * points + np.roll(points, 1, 0)
    psi = np.arctan2(d1[:, 1], d1[:, 0])
    speed = np.hypot(d1[:, 0], d1[:, 1])
    kappa = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(speed, 1e-9) ** 3
    return psi, kappa


def normals_of(points):
    psi, _ = heading_and_curvature(points)
    return np.column_stack([-np.sin(psi), np.cos(psi)])          # left-pointing


def free_extent(grid, points, normals, sign, limit=2.0):
    """How far the track surface runs from each point along +-normal."""
    steps = np.arange(0.0, limit, grid.res / 2.0)
    out = np.full(len(points), limit)
    for i, (p, n) in enumerate(zip(points, normals)):
        probe = p + sign * steps[:, None] * n
        hit = np.nonzero(~grid.is_free(probe[:, 0], probe[:, 1]))[0]
        if len(hit):
            out[i] = steps[hit[0]]
    return out


def window_min(values, half):
    """Minimum over a cyclic window. A beam slipping between two ducts reads the
    far side of the track; its neighbours along the line do not."""
    stacked = np.stack([np.roll(values, k) for k in range(-half, half + 1)])
    return stacked.min(axis=0)


def solve_box_qp(H, g, lo, hi, max_iter=200):
    """min 0.5 x'Hx + g'x  s.t.  lo <= x <= hi, by an active-set method.

    Solve with the current bound set fixed, clip, then free any bound whose
    gradient points back into the box and fix any variable that left it. The
    second-difference objective is far too badly conditioned for a first-order
    method to converge on in reasonable time; this is exact and takes a handful
    of linear solves.
    """
    n = len(g)
    at_lo = np.zeros(n, bool)
    at_hi = np.zeros(n, bool)
    x = np.zeros(n)
    for _ in range(max_iter):
        fixed = at_lo | at_hi
        x = np.where(at_lo, lo, np.where(at_hi, hi, 0.0))
        free = ~fixed
        if free.any():
            rhs = -(g[free] + H[np.ix_(free, fixed)] @ x[fixed])
            x[free] = np.linalg.solve(H[np.ix_(free, free)], rhs)
        grad = H @ x + g
        new_lo = (x < lo - 1e-9) | (at_lo & (grad > 0))
        new_hi = (x > hi + 1e-9) | (at_hi & (grad < 0))
        if np.array_equal(new_lo, at_lo) and np.array_equal(new_hi, at_hi):
            break
        at_lo, at_hi = new_lo, new_hi
    return np.clip(x, lo, hi)


def min_curvature_line(grid, reference, margin, spacing=0.2, iterations=4, smooth=1e-3):
    """Shift `reference` sideways inside the corridor to minimise sum of curvature^2."""
    line = smooth_closed(resample_closed(reference, spacing), 3)
    for _ in range(iterations):
        normals = normals_of(line)
        left = window_min(free_extent(grid, line, normals, +1.0), 2)
        right = window_min(free_extent(grid, line, normals, -1.0), 2)
        lo, hi = -(right - margin), left - margin
        # A point already closer than `margin` to a wall may only move away from it.
        lo, hi = np.minimum(lo, 0.0), np.maximum(hi, 0.0)

        n = len(line)
        D = np.zeros((n, n))                     # cyclic second difference
        idx = np.arange(n)
        D[idx, idx] = -2.0
        D[idx, (idx - 1) % n] = 1.0
        D[idx, (idx + 1) % n] = 1.0
        A = np.vstack([D * normals[:, 0], D * normals[:, 1]])
        b = np.concatenate([D @ line[:, 0], D @ line[:, 1]])
        H = A.T @ A + smooth * np.eye(n)
        g = A.T @ b
        alpha = solve_box_qp(H, g, lo, hi)
        line = resample_closed(line + alpha[:, None] * normals, spacing)
    return line


# ----------------------------------------------------------------------------
# Speed profile
# ----------------------------------------------------------------------------

def speed_profile(kappa, ds, v_max, a_lat, a_accel, a_brake):
    """Target speed at every point of a closed loop.

    Cap each point by lateral grip, v = sqrt(a_lat / |kappa|), then sweep
    forwards limiting acceleration and backwards limiting braking. The
    longitudinal budget shrinks as the lateral load grows (friction circle), so
    the car does not try to brake hard and turn hard at once. Two laps of each
    sweep make the result consistent across the start/finish seam.
    """
    n = len(kappa)
    v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(np.abs(kappa), 1e-6)))

    def budget(speed, k, a_long):
        used = min(1.0, (speed * speed * abs(k)) / a_lat)
        return a_long * math.sqrt(max(0.0, 1.0 - used * used))

    for _ in range(2):
        for j in range(n):                                   # forward: acceleration
            i, nxt = j, (j + 1) % n
            reach = math.sqrt(v[i] ** 2 + 2.0 * budget(v[i], kappa[i], a_accel) * ds)
            v[nxt] = min(v[nxt], reach)
        for j in range(n - 1, -1, -1):                       # backward: braking
            i, prv = j, (j - 1) % n
            reach = math.sqrt(v[i] ** 2 + 2.0 * budget(v[i], kappa[i], a_brake) * ds)
            v[prv] = min(v[prv], reach)
    return v


def write_raceline(path, line, v, spacing):
    psi, kappa = heading_and_curvature(line)
    s = np.arange(len(line)) * spacing
    ax = (np.roll(v, -1) ** 2 - v ** 2) / (2.0 * spacing)
    with open(path, 'w') as handle:
        handle.write('# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2\n')
        handle.write('# Minimum-curvature line generated by team_driver/raceline.py from the\n')
        handle.write('# traced grid. The driver recomputes vx from its own parameters at start-up.\n')
        for row in zip(s, line[:, 0], line[:, 1], psi, kappa, v, ax):
            handle.write('; '.join(f'{value:.4f}' for value in row) + '\n')


# ----------------------------------------------------------------------------
# Command line
# ----------------------------------------------------------------------------

def default_maps_dir():
    for candidate in ('/hackathon/maps',
                      os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'maps')):
        if os.path.isdir(candidate):
            return os.path.abspath(candidate)
    return 'maps'


def plot(path, grid, centre, line, v):
    from PIL import Image, ImageDraw                    # noqa: PLC0415 - in the image
    scale = 4
    base = np.where(grid.free, 235, np.where(grid.pixels == 0, 40, 170)).astype(np.uint8)
    img = Image.fromarray(base).convert('RGB').resize(
        (grid.width * scale, grid.height * scale), Image.NEAREST)
    draw = ImageDraw.Draw(img)

    def px(p):
        r, c = grid.to_cell(p[0], p[1])
        return (c * scale + scale // 2, r * scale + scale // 2)

    for i in range(len(centre)):
        draw.line([px(centre[i]), px(centre[(i + 1) % len(centre)])], fill=(120, 120, 255), width=1)
    lo, hi = float(v.min()), float(v.max())
    for i in range(len(line)):
        t = (v[i] - lo) / max(hi - lo, 1e-6)
        colour = (int(255 * (1 - t)), int(200 * t), 0)
        draw.line([px(line[i]), px(line[(i + 1) % len(line)])], fill=colour, width=3)
    img.save(path)


def main(argv=None):
    maps = default_maps_dir()
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--map', default=os.path.join(maps, 'icra26_compete.yaml'))
    parser.add_argument('--centerline', default=os.path.join(maps, 'icra26_compete_centerline.csv'))
    parser.add_argument('--out', default=os.path.join(maps, 'icra26_compete_raceline.csv'))
    parser.add_argument('--margin', type=float, default=0.42,
                        help='closest the line may run to a wall cell [m]')
    parser.add_argument('--spacing', type=float, default=0.2, help='point spacing [m]')
    parser.add_argument('--iterations', type=int, default=4)
    parser.add_argument('--v-max', type=float, default=7.0)
    parser.add_argument('--a-lat', type=float, default=5.0)
    parser.add_argument('--a-accel', type=float, default=4.0)
    parser.add_argument('--a-brake', type=float, default=4.0)
    parser.add_argument('--plot', help='write a PNG of the map, centreline and line')
    args = parser.parse_args(argv)

    grid = Grid(args.map)
    centre = load_xy_csv(args.centerline)[:, :2]
    line = min_curvature_line(grid, centre, args.margin, args.spacing, args.iterations)

    _, kappa = heading_and_curvature(line)
    v = speed_profile(kappa, args.spacing, args.v_max, args.a_lat, args.a_accel, args.a_brake)
    write_raceline(args.out, line, v, args.spacing)

    clear = grid.clearance(line)
    centre_len = np.sum(np.hypot(*np.diff(np.vstack([centre, centre[:1]]), axis=0).T))
    print(f'wrote {args.out}: {len(line)} points, {len(line) * args.spacing:.2f} m '
          f'(centreline {centre_len:.2f} m)')
    print(f'  max |kappa| {np.max(np.abs(kappa)):.2f} 1/m   min clearance {clear.min():.3f} m')
    print(f'  profile: {v.min():.2f}-{v.max():.2f} m/s, '
          f'ideal lap {np.sum(args.spacing / v):.2f} s at these limits')
    if args.plot:
        plot(args.plot, grid, centre, line, v)
        print(f'  plot: {args.plot}')


if __name__ == '__main__':
    main()
