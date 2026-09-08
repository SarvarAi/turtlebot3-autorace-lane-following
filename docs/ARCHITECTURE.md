# Architecture

How the node turns camera frames into velocity commands, and the specific failure
cases that shaped each design decision.

Single ROS node, `lane_driver`:

| | |
|---|---|
| **Subscribes** | `/camera/rgb/image_raw` (`sensor_msgs/Image`), queue 1, 16 MB buffer |
| **Publishes** | `/cmd_vel` (`geometry_msgs/Twist`), queue 1 |
| **Control rate** | 20 Hz (`rospy.Timer`, 0.05 s) |
| **GUI rate** | 20 Hz, main thread |

Queue depth 1 is deliberate. A backlog of stale frames is worse than a dropped one —
steering on a 300 ms-old image at 0.10 m/s means acting on the lane as it was 3 cm
ago, and the oscillation that causes is hard to diagnose because the gains look fine.

The callback does nothing but convert and store the frame. All processing happens in
the timer, so a slow frame delays one control cycle rather than backing up the queue.

---

## Stage 1 — HSV segmentation

Every frame produces three binary masks (yellow, white, red) via `cv2.inRange` in HSV.
HSV rather than BGR because separating hue from brightness is what makes a threshold
survive a shadow falling across the track.

Each mask is then cleaned:

```python
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)   # remove specks
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)   # fill gaps
```

**The failure this fixes:** floor reflections and shadows registered as lane pixels.
Open erases isolated speckle before it can form a blob large enough to pass the area
threshold; close repairs small holes inside genuine lines so one line does not
fragment into several components. Order matters — closing first would merge nearby
noise into the line before opening could remove it.

Thresholds and their rationale: **[TUNING.md](TUNING.md)**.

---

## Stage 2 — Region of interest

Lane detection runs only on the bottom **45%** of the frame (`ROI_TOP_RATIO = 0.55`).

The upper half contains the horizon, other robots, spectators, and lane markings so
distorted by perspective that their centroids are misleading. Cropping cut both the
per-frame cost and the false-positive rate.

The red mask is the exception — it is searched over the **full frame**, because the
parking square must be spotted while still distant, before it enters the ROI.

---

## Stage 3 — Connected components

We use `cv2.connectedComponentsWithStats` rather than a Hough transform. Hough
assumes straight segments; on this track's curves it produced fragmented, jittering
lines and needed per-section retuning. Blob centroids degrade far more gracefully:
a partially visible curved line still yields a usable centroid.

Three different extraction strategies run on the three masks.

### Yellow — paired line detection

`find_lines_in_mask()` keeps blobs above `MIN_PIXELS` (200 px), then:

1. **No blobs** → report nothing visible.
2. **One blob** → classify it as left or right (see below).
3. **Two or more** → take the two largest, and require them to be at least
   `MIN_LINE_SEPARATION` (200 px) apart.

> **Failure case:** a sharply curved yellow line splits into two blobs. Naively,
> that reads as two lane boundaries, so the computed lane center lands *on the line
> itself* and the robot drives straight over it. The separation check rejects the
> pair and falls back to single-line handling.

### Single-line disambiguation

When one line is visible, deciding whether it is the left or right boundary
determines the sign of the steering correction — getting it wrong steers the robot
off the track at full correction.

A midpoint test alone is not enough: on a sharp right curve, the *right* boundary
sweeps across the image center and appears on the left. `_classify_single()` uses
the last known side as a prior:

```python
if cx < midpoint:
    if last_side == 'right' and cx > midpoint * 0.4:
        return None, int(cx)     # still the right line, mid-curve
    return int(cx), None         # genuinely the left line
```

The `0.4` / `1.6` bands are the hysteresis: a line must cross well past the center,
not merely touch it, before the classification flips.

Once classified, the lane center is inferred at a fixed `LANE_OFFSET` (160 px) from
the visible line — a constant that only holds because the camera pitch is fixed.

### White — dash clustering

Dashed highway markings defeat both of the above: each dash is its own component, and
which dashes are visible changes every frame.

`find_white_lines_clustered()` splits blobs by image midpoint and takes an
**area-weighted centroid** of each side:

```python
left_x = sum(cx * area for cx, area in left_blobs) / sum(area for _, area in left_blobs)
```

Weighting by area matters: near dashes are large and geometrically reliable, distant
ones are small and perspective-distorted. Weighting lets the near dashes dominate
without discarding the far ones outright.

### Red — largest blob

`find_red_square()` returns the centroid and area of the largest component above
`MIN_RED_PIXELS` (500 px). Area doubles as a distance estimate, which is what drives
the parking approach.

---

## Stage 4 — Smoothing

Two independent moving averages sit between detection and actuation:

| Window | Frames | Purpose |
|---|---|---|
| `TARGET_SMOOTH_WIN` | 4 | Absorbs target jumps when a dash appears or disappears |
| `ERROR_SMOOTH_WIN` | 3 | Feeds the derivative term a signal it can differentiate |

The second one is the important one. A raw frame-to-frame error difference is mostly
detection noise, and multiplying that by `Kd` injects noise straight into the steering
command — the derivative term made things *worse* until the error was smoothed first.

Both histories are cleared on every state transition (`reset_pd()`), so a stale error
from yellow-lane driving cannot produce a phantom correction on the first frame of
highway mode.

---

## Stage 5 — PD control

```python
derivative = smoothed_error - self.last_error
angular    = -(Kp * smoothed_error + Kd * derivative)
angular    = clamp(angular, ±MAX_ANGULAR)
```

No integral term. Steady-state lane offset is not a problem worth solving here, and
an integrator would wind up during the seconds when a line is occluded and then
discharge as a lurch. P and D only.

A `DEADBAND` of 10 px zeroes the command near center, so the robot tracks straight
sections without twitching.

**Error-adaptive velocity** is what actually produced the zero-crossing result:

```python
speed_factor = min(1.0, abs_error / 150.0)
linear = MAX_SPEED - (MAX_SPEED - MIN_SPEED) * speed_factor
```

Large error means a sharp curve, so the robot slows to 0.04 m/s, which both tightens
the achievable turn radius and buys more frames of processing per meter travelled.
Straight sections run at the full 0.10 m/s.

---

## Stage 6 — Mission state machine

```
LANE_YELLOW ⇄ LANE_WHITE ──► TURN_LEFT ──► SEEK_RED ──► PARK ──► FINAL_PUSH ──► DONE
```

### Yellow ⇄ white arbitration

Both modes are always eligible, so a hysteresis lock prevents flapping at the
boundary between sections:

- `white_count >= 2` dashes **and** not both yellow lines visible → white mode,
  lock refreshed to 8 frames
- both yellow lines visible → yellow mode immediately, lock cleared
- otherwise → stay in white while the lock counts down

Yellow wins ties. Two clean yellow boundaries is the strongest evidence available,
and stray white reflections should never override it.

### The mission lock

`SEEK_RED` and `PARK` would end the run if triggered early, and red objects are not
rare in a classroom. So the final stages stay locked until the robot has proven it
drove the highway: **15 cumulative frames** in white mode sets
`final_stage_unlocked`. Before that flag, no T-junction detection and no red overlay.

An `UNLOCK_COOLDOWN_FRAMES` (10) grace period covers the moment of leaving white
mode, when lines are legitimately missing for a few frames and would otherwise read
as a T-junction.

### T-junction detection

The junction is recognized by **absence**: 25 consecutive frames (~1.25 s) with no
yellow lines *and* no white dashes. There is nothing distinctive to detect at the end
of the road, but the total disappearance of all lane markings is unambiguous — once
gated behind the mission lock.

### The left turn

The only maneuver that cannot be run open-loop. A fixed arc that works at full
battery undershoots as voltage drops, and overshooting means crossing the lane.

**Phase A (1.0 s, open-loop):** drive at 0.05 m/s with 0.5 rad/s yaw. Committing
blind is necessary because at the moment of detection there are no lines left to
servo on.

**Phase B (≤3.0 s, closed-loop):** as yellow lines re-enter view, hold them outside a
±100 px safety corridor around image center:

```python
if y_right is not None and (y_right - image_center) < YELLOW_SAFETY_DIST:
    avoid_correction += Kp_TURN_AVOID * (YELLOW_SAFETY_DIST - (y_right - image_center))
```

Right line too close → turn harder left. Left line too close → ease off. Inside 60 px
the linear velocity drops to 30%, which converts the maneuver into a near-stationary
rotation instead of an arc that would carry the robot over the line.

Phase B ends on success (both lines visible, ≥250 px apart) or on the 3 s timeout.
Both transition to `SEEK_RED` — a timeout still leaves the robot roughly aligned, and
`SEEK_RED` can recover from that, whereas stopping cannot.

### Parking

`SEEK_RED` cruises at 0.07 m/s, steering proportionally toward the red centroid, until
red area exceeds 8000 px → `PARK`.

`PARK` closes in at 0.04 m/s and stops on **any** of three conditions:

| Condition | Meaning |
|---|---|
| area ≥ 35000 px | Square fills the frame |
| centroid y ≥ 90% of frame height | Square has reached the bottom edge |
| red disappears entirely | Robot is on top of it, square is under the chassis |

The third is the one that made parking reliable. As the robot arrives, the square
slides out of the camera's field of view — losing the target is the *success*
signal, not a failure, and treating it as failure was what stalled the robot short
of the square in earlier runs.

`FINAL_PUSH` then drives blind for 1.5 s at 0.05 m/s to seat the robot fully on the
square, and `DONE` latches zero velocity.

---

## Safety

- `rospy.on_shutdown(stop_robot)` publishes zero velocity **five times** at 20 ms
  intervals. One message on a dropped Wi-Fi packet is not enough, and the failure
  mode is a robot driving away with no controller attached.
- `MAX_ANGULAR` (1.5 rad/s) clamps every angular command, including turn-avoidance.
- `required="true"` in the launch file tears down the whole launch if the node dies.
- Q in any debug window triggers a clean `rospy.signal_shutdown`.
