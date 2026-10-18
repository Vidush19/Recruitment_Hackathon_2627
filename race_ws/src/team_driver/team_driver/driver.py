#!/usr/bin/env python3
"""Racing-line follower: pure pursuit on a minimum-curvature line, with a speed
profile and a closed-loop speed controller.

    ros2 run team_driver driver
    ros2 launch team_driver driver.launch.py

The pieces, in the order the data flows through them:

1. **The line** (`raceline.py`, offline). A minimum-curvature line computed from
   the traced grid: wide in, apex, wide out. Loaded from
   `maps/icra26_compete_raceline.csv`, or regenerated at start-up if missing.

2. **The speed profile** (start-up). Curvature -> lateral-grip speed cap, then
   forward/backward passes for acceleration and braking, so every corner is
   approached already at its speed. Its limits are parameters below.

3. **Steering** (every scan). Pure pursuit from the rear axle to a point a
   speed-dependent distance down the line - from where the car *will* be
   when the command takes effect, not where it was when the scan was taken.
   The simulator delivers a frame every 0.1 s of simulated time on a
   laptop-class machine, and a command acts on the next frame, so without
   that projection the car reacts half a metre late at 5 m/s and weaves.

4. **Throttle** (every scan). Measured on the compete build, the motor acts
   as a rate-limited speed servo: a constant throttle u settles at
   v = 24.6 * u m/s (0.08 -> 1.99 m/s, 0.15 -> 3.67, 0.2 -> 4.87), getting
   there at about 5 m/s^2 either way - throttle 0 slows the car as hard as
   it speeds up. So the feed-forward is u = v_target / 24.6, a PI trim
   absorbs cornering drag, and negative throttle is never needed: it brakes
   no harder and risks locking the wheels.

5. **Recovery.** After a collision the simulator respawns the car at a
   checkpoint, stopped, and not always beside the line. The driver re-finds
   the line by position *and* heading (distance alone can pick a stretch of
   line on the far side of a wall), caps its speed for a moment, and backs
   off if it is ever pinned against a duct.

A LiDAR guard - slow down if something is closer ahead than the car can stop
for - is implemented but off by default (`guard_enabled`); with the line and
profile tuned it only ever trimmed speed in fast corners.

Pose comes from `/odom`, which the hackathon rules allow (rule 35). A judged
run passes no parameter file, so the defaults declared below are the tuned
values that race.
"""

import csv
import math
import os

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32
from visualization_msgs.msg import Marker, MarkerArray

from team_driver import raceline as rl

# The steering command is normalised; this is what +-1.0 actually means.
MAX_STEERING_RAD = 0.5236
WHEELBASE = 0.324          # [m]


def devkit_qos() -> QoSProfile:
    """Match the QoS the AutoDRIVE bridge publishes and subscribes with.

    RELIABLE / KEEP_LAST(1) / VOLATILE. Get this wrong and the topics simply do
    not connect: a BEST_EFFORT subscriber will never see a RELIABLE publisher's
    messages, and `ros2 topic list` will still cheerfully show the topic.
    """
    return QoSProfile(
        durability=QoSDurabilityPolicy.VOLATILE,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class SpeedController:
    """Target speed -> normalised throttle.

    throttle = ff(v_target) + kp * error + ki * integral, where ff is the
    steady-state throttle that holds v_target (fitted to constant-throttle
    runs). The integral is clamped so one slow corner cannot wind it up into
    the next straight. Overspeed beyond `brake_margin` drops to
    `-max_brake` (0 by default: the motor already brakes at ~5 m/s^2 on a
    closed throttle) and freezes the integral rather than letting it unwind.
    """

    def __init__(self, ff0, ff1, kp, ki, i_limit, brake_margin, brake_gain, max_brake):
        self.ff0, self.ff1 = ff0, ff1
        self.kp, self.ki, self.i_limit = kp, ki, i_limit
        self.brake_margin, self.brake_gain, self.max_brake = brake_margin, brake_gain, max_brake
        self.integral = 0.0

    def feed_forward(self, v):
        return self.ff0 + v / self.ff1

    def step(self, v_target, v, dt):
        error = v_target - v
        if error < -self.brake_margin:
            return max(-self.max_brake, self.brake_gain * (error + self.brake_margin))
        self.integral = float(np.clip(self.integral + error * dt, -self.i_limit, self.i_limit))
        throttle = self.feed_forward(v_target) + self.kp * error + self.ki * self.integral
        return float(np.clip(throttle, 0.0, 1.0))


class Driver(Node):

    def __init__(self):
        super().__init__('driver')

        p = self.declare_parameter
        p('vehicle_ns', '/autodrive/roboracer_1')
        p('raceline_csv', '/hackathon/maps/icra26_compete_raceline.csv')
        p('map_yaml', '/hackathon/maps/icra26_compete.yaml')
        p('centerline_csv', '/hackathon/maps/icra26_compete_centerline.csv')
        p('line_margin', 0.42)       # [m] used only if the line has to be regenerated

        # Speed profile
        # Tuned on the compete build: a_lat 9 ran 13 laps clean at ~10.7 s,
        # 10 hit the top hairpin, 12 left the track. See SUBMISSION.md.
        p('v_max', 11.0)             # [m/s] straight-line cap (the straights top out ~9.5)
        p('a_lat', 9.0)              # [m/s^2] cornering grip
        p('a_accel', 5.0)            # [m/s^2] measured motor limit, ~5 either way
        p('a_brake', 4.0)            # [m/s^2] below the motor's ~5: frames and lag eat the rest
        p('speed_scale', 1.0)        # global multiplier on the profile
        p('v_min', 0.8)              # [m/s] never ask for less than this
        p('speed_preview_s', 0.3)    # [s] look this far ahead in the profile

        # Pure pursuit
        p('lookahead_base', 0.45)    # [m]
        p('lookahead_gain', 0.25)    # [s] extra lookahead per m/s
        p('lookahead_min', 0.55)     # [m]
        p('lookahead_max', 1.8)      # [m]
        p('steer_limit', 1.0)        # fraction of full lock
        p('reacquire_heading_m', 1.5)  # [m/rad] heading weight when re-finding the line
        p('latency_frames', 1.0)     # commands act one simulator frame late...
        p('latency_extra_s', 0.02)   # [s] ...plus this
        p('steer_a_lat', 12.0)       # [m/s^2] never ask for more lateral accel than this

        # Speed controller
        p('ff0', 0.0)                # throttle offset
        p('ff1', 24.6)               # [m/s] steady-state speed per unit throttle
        p('kp', 0.02)                # throttle per m/s of error
        p('ki', 0.04)
        p('i_limit', 0.5)            # [m/s * s]
        p('brake_margin', 1.0)       # [m/s] overspeed before closing the throttle fully
        p('brake_gain', 0.0)
        p('max_brake', 0.0)          # never reverse the motor

        # Stuck recovery: stationary under throttle this long -> back off
        p('stuck_after_s', 1.0)      # [s] simulated
        p('reverse_for_s', 0.6)      # [s] simulated
        p('reverse_throttle', -0.04)  # ~1 m/s backwards

        # After a collision the simulator respawns the car, stopped and not
        # always pointing down the line. Come away from that gently, so one
        # contact cannot chain into several.
        p('after_hit_v', 3.0)        # [m/s]
        p('after_hit_s', 1.5)        # [s] simulated

        # LiDAR guard. Off by default: with the line and profile tuned it only
        # ever caps speed in fast corners, where the wall ahead is legitimately
        # close. Kept for running on a changed track.
        p('guard_enabled', False)
        p('guard_half_width', 0.17)  # [m] corridor ahead of the car to check
        p('guard_decel', 3.0)        # [m/s^2] assumed stopping deceleration
        p('guard_buffer', 0.25)      # [m] distance always kept

        # Development
        p('throttle_override', -9.0)  # >= -1 drives open loop at this throttle
        p('log_csv', '')              # write a per-scan log here

        g = lambda name: self.get_parameter(name).value          # noqa: E731
        self.cfg = {name: g(name) for name in (
            'speed_scale', 'v_min', 'speed_preview_s',
            'lookahead_base', 'lookahead_gain', 'lookahead_min', 'lookahead_max',
            'steer_limit', 'reacquire_heading_m', 'latency_frames', 'latency_extra_s',
            'steer_a_lat',
            'stuck_after_s', 'reverse_for_s', 'reverse_throttle', 'after_hit_v', 'after_hit_s',
            'guard_enabled', 'guard_half_width', 'guard_decel', 'guard_buffer',
            'throttle_override')}
        self.controller = SpeedController(
            g('ff0'), g('ff1'), g('kp'), g('ki'), g('i_limit'),
            g('brake_margin'), g('brake_gain'), g('max_brake'))

        self.load_line(g('raceline_csv'), g('map_yaml'), g('centerline_csv'), g('line_margin'))
        self.ds = float(np.mean(np.hypot(*np.diff(np.vstack([self.line, self.line[:1]]), axis=0).T)))
        self.psi, self.kappa = rl.heading_and_curvature(self.line)
        self.v_profile = rl.speed_profile(self.kappa, self.ds, g('v_max'), g('a_lat'),
                                          g('a_accel'), g('a_brake'))
        self.get_logger().info(
            f'line: {len(self.line)} pts, {len(self.line) * self.ds:.1f} m; profile '
            f'{self.v_profile.min():.2f}-{self.v_profile.max():.2f} m/s, '
            f'ideal lap {np.sum(self.ds / self.v_profile):.2f} s')

        ns = str(g('vehicle_ns')).rstrip('/')
        qos = devkit_qos()
        self.throttle_pub = self.create_publisher(Float32, f'{ns}/throttle_command', qos)
        self.steering_pub = self.create_publisher(Float32, f'{ns}/steering_command', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/driver/markers', 1)
        self.create_subscription(LaserScan, f'{ns}/lidar', self.scan_callback, qos)
        self.create_subscription(Odometry, f'{ns}/odom', self.odom_callback, qos)
        self.create_subscription(Float32, f'{ns}/lap_time', self.lap_time_callback, qos)
        self.laps = self.collisions = 0
        self.create_subscription(Int32, f'{ns}/lap_count',
                                 lambda m: setattr(self, 'laps', m.data), qos)
        self.create_subscription(Int32, f'{ns}/collision_count', self.collision_callback, qos)
        self.calm_until = -1.0

        self.position = None
        self.yaw = 0.0
        self.speed = 0.0
        self.index = None             # nearest line point, tracked between scans
        self.last_steering = 0.0
        self.frame_dt = 0.1           # [s] simulated, smoothed
        self.stuck_since = None
        self.reverse_until = -1.0
        # Simulator clock. The bridge stamps messages with wall time, and how
        # many frames a simulated second gets depends on the machine (about 11
        # on a laptop, more on the judging machine), so the controller's dt
        # comes from the simulator's own lap clock instead.
        self.sim_time = 0.0           # accumulated across laps
        self._lap_clock = None
        self.last_time = None
        self._marker_divisor = 0

        self.log = None
        if g('log_csv'):
            self.log = open(g('log_csv'), 'w', newline='')
            self.log_writer = csv.writer(self.log)
            self.log_writer.writerow(['t', 'x', 'y', 'yaw', 'v', 'idx', 'v_target',
                                      'throttle', 'steering', 'guard', 'laps', 'collisions'])

    # ------------------------------------------------------------------
    def load_line(self, path, map_yaml, centre_csv, margin):
        if os.path.isfile(path):
            self.line = rl.load_raceline(path)
            return
        self.get_logger().warn(f'{path} not found - generating the line from the map')
        grid = rl.Grid(map_yaml)
        centre = rl.load_xy_csv(centre_csv)[:, :2]
        self.line = rl.min_curvature_line(grid, centre, margin)

    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def collision_callback(self, msg):
        if msg.data > self.collisions:
            self.calm_until = self.sim_time + self.cfg['after_hit_s']
        self.collisions = msg.data

    def lap_time_callback(self, msg):
        t = float(msg.data)
        if not math.isfinite(t):
            return
        if self._lap_clock is not None and t >= self._lap_clock:
            self.sim_time += t - self._lap_clock
        self._lap_clock = t

    # ------------------------------------------------------------------
    # The control loop, once per LiDAR scan.
    # ------------------------------------------------------------------
    def scan_callback(self, scan):
        now = self.sim_time
        dt = 0.025 if self.last_time is None else now - self.last_time
        dt = min(max(dt, 0.0), 0.2)
        self.last_time = now

        if self.position is None:
            self.publish(0.0, 0.0)
            return

        c = self.cfg
        if 0.0 < dt < 0.2:
            self.frame_dt += 0.1 * (dt - self.frame_dt)
        x, y, yaw = self.predict(c['latency_frames'] * self.frame_dt + c['latency_extra_s'])
        self.index = self.nearest(x, y, yaw)

        lookahead = float(np.clip(c['lookahead_base'] + c['lookahead_gain'] * self.speed,
                                  c['lookahead_min'], c['lookahead_max']))
        target = self.point_ahead(self.index, x, y, lookahead)
        steering = self.pure_pursuit(x, y, yaw, target)
        self.last_steering = steering

        v_target = self.target_speed(self.index)
        guard = self.lidar_guard(scan) if c['guard_enabled'] else math.inf
        v_target = max(c['v_min'], min(v_target, guard))
        if self.sim_time < self.calm_until:
            v_target = min(v_target, c['after_hit_v'])

        if c['throttle_override'] >= -1.0:
            throttle = c['throttle_override']
        else:
            throttle = self.controller.step(v_target, self.speed, dt)
        steering, throttle = self.recover(steering, throttle)
        self.publish(steering, throttle)

        if self.log:
            self.log_writer.writerow([f'{now:.3f}', f'{x:.3f}', f'{y:.3f}', f'{self.yaw:.3f}',
                                      f'{self.speed:.3f}', self.index, f'{v_target:.3f}',
                                      f'{throttle:.3f}', f'{steering:.3f}',
                                      f'{guard:.2f}' if guard < 50 else 'inf',
                                      self.laps, self.collisions])

        self._marker_divisor = (self._marker_divisor + 1) % 8
        if self._marker_divisor == 0:
            self.publish_markers(target)

    def recover(self, steering, throttle):
        """Back away from whatever the car is pinned against.

        A car that is commanding throttle and not moving is against a duct.
        Reverse with the steering mirrored for a moment, then hand back. The
        simulator usually respawns a car after contact, so this is for the
        case it does not - without it the run ends as DNF_STUCK, or as a
        disqualification counted out one collision at a time.
        """
        c = self.cfg
        now = self.sim_time
        if now < self.reverse_until:
            return -steering, c['reverse_throttle']
        if throttle > 0.05 and self.speed < 0.15:
            if self.stuck_since is None:
                self.stuck_since = now
            elif now - self.stuck_since > c['stuck_after_s']:
                self.get_logger().warn('stuck - reversing')
                self.reverse_until = now + c['reverse_for_s']
                self.stuck_since = None
                self.controller.integral = 0.0
        else:
            self.stuck_since = None
        return steering, throttle

    def predict(self, horizon):
        """Kinematic bicycle roll-forward of the odom pose under the last command."""
        x, y = self.position
        yaw, v = self.yaw, self.speed
        rate = v * math.tan(self.last_steering * MAX_STEERING_RAD) / WHEELBASE
        steps = 4
        h = horizon / steps
        for _ in range(steps):
            x += v * math.cos(yaw) * h
            y += v * math.sin(yaw) * h
            yaw += rate * h
        return x, y, yaw

    def nearest(self, x, y, yaw):
        """Where on the line the car is.

        While tracking, only a short window ahead of the last index is
        searched, so the car can never jump to a part of the line it happens
        to pass close to. When that fails - the car is far from the line,
        which after a collision means the simulator has respawned it at a
        checkpoint - the whole line is searched, and candidates are scored by
        distance *and* by how far the line's direction there disagrees with
        the car's heading. Distance alone picks the closest stretch of line,
        which from the respawn point beside the diagonal is on the other side
        of a wall; the car then drives into that wall until disqualified.
        """
        n = len(self.line)
        if self.index is not None:
            candidates = (self.index + np.arange(-5, 40)) % n
            d = np.hypot(self.line[candidates, 0] - x, self.line[candidates, 1] - y)
            if d.min() < 1.0:
                return int(candidates[np.argmin(d)])
        d = np.hypot(self.line[:, 0] - x, self.line[:, 1] - y)
        mismatch = np.abs((self.psi - yaw + math.pi) % (2.0 * math.pi) - math.pi)
        return int(np.argmin(d + self.cfg['reacquire_heading_m'] * mismatch))

    def point_ahead(self, index, x, y, distance):
        """First point down the line at least `distance` from the car, interpolated."""
        n = len(self.line)
        prev = self.line[index]
        for k in range(1, n):
            cur = self.line[(index + k) % n]
            d_cur = math.hypot(cur[0] - x, cur[1] - y)
            if d_cur >= distance:
                d_prev = math.hypot(prev[0] - x, prev[1] - y)
                t = 0.0 if d_cur == d_prev else (distance - d_prev) / (d_cur - d_prev)
                t = min(max(t, 0.0), 1.0)
                return prev + t * (cur - prev)
            prev = cur
        return self.line[(index + 1) % n]

    def pure_pursuit(self, x, y, yaw, target):
        dx, dy = target[0] - x, target[1] - y
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        local_x = cos_y * dx + sin_y * dy
        local_y = -sin_y * dx + cos_y * dy
        dist2 = max(local_x * local_x + local_y * local_y, 1e-6)
        curvature = 2.0 * local_y / dist2
        angle = math.atan(WHEELBASE * curvature)
        # At speed, a lock the tyres cannot hold only slides the car sideways,
        # and on a 10 Hz loop that turns into a frame-to-frame oscillation.
        # Cap the request at what `steer_a_lat` of lateral acceleration needs.
        v2 = max(self.speed, 1.0) ** 2
        angle_cap = math.atan(WHEELBASE * self.cfg['steer_a_lat'] / v2)
        angle = float(np.clip(angle, -angle_cap, angle_cap))
        limit = self.cfg['steer_limit']
        return float(np.clip(angle / MAX_STEERING_RAD, -limit, limit))

    def target_speed(self, index):
        c = self.cfg
        n = len(self.line)
        ahead = int(max(1, math.ceil(self.speed * c['speed_preview_s'] / self.ds)))
        window = self.v_profile[(index + np.arange(ahead + 1)) % n]
        return float(window.min()) * c['speed_scale']

    def lidar_guard(self, scan):
        """Highest speed from which the car can still stop short of whatever is
        straight ahead in a car-width corridor."""
        ranges = np.nan_to_num(np.asarray(scan.ranges), nan=0.0, posinf=10.0, neginf=0.0)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        front = np.abs(angles) < 0.6
        r, a = ranges[front], angles[front]
        valid = r > 0.07
        lateral = np.abs(r * np.sin(a))
        ahead = r * np.cos(a)
        hit = valid & (lateral < self.cfg['guard_half_width']) & (ahead > 0)
        if not hit.any():
            return math.inf
        free = max(0.0, float(ahead[hit].min()) - self.cfg['guard_buffer'])
        return math.sqrt(2.0 * self.cfg['guard_decel'] * free)

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------
    def publish(self, steering, throttle):
        self.steering_pub.publish(Float32(data=float(np.clip(steering, -1.0, 1.0))))
        self.throttle_pub.publish(Float32(data=float(np.clip(throttle, -1.0, 1.0))))

    def publish_markers(self, target):
        stamp = self.get_clock().now().to_msg()
        aim = Marker()
        aim.header.frame_id = 'world'
        aim.header.stamp = stamp
        aim.ns, aim.id = 'team_driver', 0
        aim.type, aim.action = Marker.SPHERE, Marker.ADD
        aim.pose.position.x, aim.pose.position.y = float(target[0]), float(target[1])
        aim.pose.orientation.w = 1.0
        aim.scale.x = aim.scale.y = aim.scale.z = 0.15
        aim.color.r, aim.color.g, aim.color.a = 1.0, 0.3, 1.0
        array = MarkerArray()
        array.markers.append(aim)
        self.marker_pub.publish(array)

    def destroy_node(self):
        if self.log:
            self.log.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
