# Team Chitti 3.0

## Members
| Name | Matric number | Email |
| VIDUSH GUPTA | U2523505J | VIDUSH001@E.NTU.EDU.SG |
| RAMESH BABU RISHI | U2523684K | RISHI008@E.NTU.EDU.SG |
| SHIVAKUMAR ARAVIND  | U2521979G | ARAVIND016@E.NTU.EDU.SG |

## Approach
The driver follows a racing line it generates from the map. Offline,
`team_driver/raceline.py` takes the traced centreline and the occupancy grid,
measures how far the track extends either side of every point, and slides each
point sideways to minimise the summed squared curvature of the closed loop,
keeping 0.42 m off the boundary. That gives wide in, apex, wide out, and a
52.6 m lap against the centreline's 54.4 m. It is a box-constrained least-squares
problem in the lateral offsets (Heilmeier et al.'s minimum-curvature line),
solved with a small active-set method in plain numpy because the image has no
scipy, and re-linearised four times because the normals move with the line. 
We saw that using the flexibility of the competition, using LIDAR proved to be slower unless an obstacle was introduced which again is outside the scope of the competition. We aimed to be the fastest under the constraint of the competition.

At start-up the driver turns the line's curvature into a target speed for every
point: a lateral-grip cap (a_lat = 9 m/s²), then forward and backward passes
for acceleration and braking, so the car arrives at each corner already at its
speed. Every frame it steers by pure pursuit, with two additions that turned
out to matter more than the algorithm. **Latency compensation:** on our machine
the simulator delivers one frame per 0.1 s of simulated time and a command acts
on the next frame, so the driver rolls its pose forward one frame with a
bicycle model before steering. That alone took the mean tracking error from
0.21 m to 0.06 m and stopped the car weaving. **A speed-dependent steering
cap:** above about 8 m/s, a lock the tyres cannot hold made the car slide
sideways and oscillate frame to frame; the steering is now limited to what
12 m/s² of lateral acceleration needs at the current speed.

What did not work, or was rejected:
 **A shorter lookahead at low speed** (to stop pure pursuit cutting the hairpins).
  It made the car oscillate at low speed.
- **A higher steering cap.** It let the car into the open area left of the diagonal
  wall.
- **The LiDAR guard** (slow down if the wall ahead is closer than the stopping
  distance). It is implemented but off by default, because with the line tuned it
  only trimmed speed in fast corners where the wall ahead is legitimately close.

## How it uses the inputs
- **LiDAR:** triggers the control loop (one control step per scan). Its ranges feed
  only the optional guard, which is off by default.
- **Pose:** the simulator's ground truth on `…/odom` (position, yaw, speed). We did
  not write our own localisation.
- **`…/lap_time`:** used as the simulator clock. The bridge stamps messages with wall
  time, and the number of frames per simulated second depends on the machine, so
  the controller's dt and the latency estimate come from it.
- **`…/collision_count`:** after a contact, the target speed is capped at 3 m/s for
  1.5 s, so a respawn at an awkward angle cannot chain into more hits.
- **Camera, IMU, encoders:** not used.

## Speed control
We measured the motor before writing a controller. On the compete build it acts
as a rate-limited speed servo rather than a torque source. A constant throttle u
settles at v ≈ 24.6·u m/s (0.08 → 1.99 m/s, 0.15 → 3.67, 0.2 → 4.87), and the
car gets there at about 5 m/s² in either direction. Throttle 0 slows the car as
hard as it accelerates, and −0.3 brakes no harder before it starts reversing.

So the controller is:
- **Feed-forward:** u = v_target / 24.6.
- **PI trim:** a small correction (kp = 0.02, ki = 0.04, integral clamped) for the
  speed lost in corners.
- **No negative throttle.** It brakes no harder and risks locking the wheels.

The speed profile assumes 4 m/s² of braking rather than the motor's ~5. The
difference absorbs the frame delay, after we saw the car arrive 1.5 m/s too fast
at the lower-left U-turn. The target is the minimum of the profile over the next
0.3 s.

## Results on our own machine
The runs committed in `results/submitted/`: `./scripts/evaluate.sh --team "Chitti 3.0" --runs 3`
on the final code, unedited.

| Run | Status | Best lap | 10-lap total | Collisions |
| --- | --- | --- | --- | --- |
| r1 | COMPLETE | 11.271 | 113.102 | 0 |
| r2 | COMPLETE | **11.238** | **112.884** | 0 |
| r3 | COMPLETE | 11.287 | 113.118 | 0 |

We ran three more 3-run evaluations during development (not committed, same
driver apart from the clean-ups). One with the final settings gave 3/3 COMPLETE,
0 collisions, best 11.134 / 112.410. One at the earlier, more aggressive settings
(a_brake 5, 0.32 m margin) gave COMPLETE with 2 collisions and a 129.2 s total.
One before the respawn fix was disqualified (below).

### The machine that produced them
| CPU | GPU | RAM | OS | Typical real-time factor |
| --- | --- | --- | --- | --- |
| Intel Core Ultra 7 256V (8 cores) | Intel Arc 140V integrated, not used (headless) | 16 GB (about 7.6 GB visible to WSL) | Windows 11 Home, WSL 2 Ubuntu 24.04, Docker Engine in WSL | 1.00 on the committed runs; 0.92 on earlier days |

**One thing that may explain a gap between your numbers and ours: control rate.**
On this machine the simulator delivers one frame per 0.1 s of simulated time,
about 10 control steps per simulated second. A frame takes longer than 0.1 s of
wall time to produce, so Unity caps its per-frame step. The judging machine will
very likely deliver more frames per simulated second.

The driver is built for that. Its delay estimate is one measured frame plus
0.02 s, not a constant, and the controller's dt comes from the simulator clock.
More frames should mean tighter tracking and, if anything, a slightly faster
lap. But all our tuning was done at about 10–17 frames per second, so if your
times differ a lot, that is the first thing we would look at.

## Tuning record
Three-lap runs (later ones longer) in our own harness. Grip limit a_lat and top
speed are the speed-profile parameters; collisions at the start come from
resetting the car and are excluded.

| a_lat (m/s²) | v_max (m/s) | Clean-lap time | Collisions |
| --- | --- | --- | --- |
| 3 | 5 | 17.0 s | 0 |
| 5 | 7 | 13.5 s | 0 |
| 6 | 8 | 12.4 s | 0 |
| 7 | 9 | 11.6 s | 0, after adding the steering cap; 2 before it |
| 8 | 10 | 11.1 s | 0 over 12 laps |
| 9 | 11 | 10.7 s | 0 over 13 laps; 2 in one scored 10-lap run |
| 10 | 11 | 10.4 s | 8 (top hairpin, centre S) |
| 12 | 11 | — | car left the track |

The final settings (a_lat 9, a_brake 4, 0.42 m margin) give up about 0.4 s a
lap against the fastest clean configuration. A collision costs 10 s, so we kept
the margin.

## Anything precomputed
**`maps/icra26_compete_raceline.csv`**, the racing line (`s; x; y; psi; kappa; vx; ax`).
- **Produced by** `team_driver/raceline.py` from `maps/icra26_compete.yaml` and
  `maps/icra26_compete_centerline.csv`, both supplied by the organisers.
- **To regenerate,** inside the container: `ros2 run team_driver make_raceline`
  (add `--plot /tmp/line.png` to see it).
- **Deterministic:** regenerating gives a byte-identical file.
- **Fallback:** if the file is missing, the driver regenerates the line at start-up.
- **Speed column ignored:** the driver recomputes speeds from its own parameters,
  so the `vx` column is for inspection only.

No trained weights.

## Dependencies we added
None. `raceline.py` uses numpy and PyYAML, which the image already provides, and
Pillow only for the optional `--plot`.

| Package | pip or apt | What we use it for |
| --- | --- | --- |
| none | | |

## Third-party code and references
- Heilmeier et al., *Minimum curvature trajectory planning and control for an
  autonomous race car*, Vehicle System Dynamics, 2019. The formulation of the line
  optimisation; our solver and code are our own.
- Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*,
  CMU-RI-TR-92-01.
- The organisers' traced occupancy grid and centreline in `maps/`, and the
  `driver.py` template (the QoS profile and node skeleton are kept from it).

## AI assistance
Claude Code (Anthropic) was used to think along with. It helped
verify `raceline.py` and `driver.py`, ran the system-identification and
parameter-sweep experiments in the simulator, analysed the logs, and drafted this
file. Every idea is original and generated by a team member, CLAUDE didn't generate a single idea on its own. Every design decision above is backed by a measurement we can show and
explain.

## Known issues
- **Rejoining after a crash depends on the simulator's checkpoints.** After a
  contact, the simulator respawns the car at a checkpoint. One of them is in the
  open area left of the diagonal wall, about 2.5 m from the line. Re-finding the
  line by distance alone picked a stretch on the far side of the wall, and the car
  drove into it until disqualified; that is what disqualified one development run.
  The driver now scores candidates by distance and heading, which picks the right
  stretch from that point. But it is only tested on the respawn points we actually
  hit.
- **Speed is limited by two or three corners.** The grip limit is one number for the
  whole lap, so it is set by the top hairpin (index ~160), the entry to the centre
  S (~130) and the lower-left U-turn (~180). With more time:
  - make the grip limit speed- or curvature-dependent, measured from logged lateral
    acceleration;
  - add a curvature feed-forward to the steering so pure pursuit stops cutting the
    hairpins;
  - vary the wall margin with curvature (the car's corners only swing wide in tight
    turns);
  - send the speed target from where the car will be once the motor has responded,
    rather than the minimum over a window.
- **Tuning was done at about 10 frames per second** (see the machine section).

### Possible bugs in the judging environment (rule 49)
- **Lap-time mismatch:** in run r3, `simulator_best_lap_time` (11.2759) does not equal
  `fastest_raw_lap_time` (11.2869), which §5.3 says should never happen. It is a
  0.011 s difference, and it is in the committed file
  `results/submitted/Chitti_3_0__20261018T052600Z_r3.json`.
- **scipy is not installed:** rule 37 and §5.8 list it as preinstalled, but
  `import scipy` fails in the image built from this repository.
- **Docs and code disagree** in four places:
  - §2.9 and `maps/README.md` say `pure_pursuit` follows the centreline, but no such
    node ships.
  - §2.4 says `devkit_qos()` is in `control.py`; it is in `driver.py`.
  - Rule 35 says there is no shipped map, but there is one in `maps/`.
  - The `SUBMISSION.md` template mentions a "shipped SpeedController", which does
    not exist.
