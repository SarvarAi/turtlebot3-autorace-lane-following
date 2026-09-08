# Parameter Tuning

Every constant in `lane_follower.py`, what it does, and why it holds that value.
All parameters live at the top of `LaneDriver.__init__`.

These values were tuned **on the physical robot and track**, not in simulation — the
available laptops could not run Gazebo fast enough for the timing to be meaningful.
That makes the thresholds honest but narrow: the HSV values in particular encode the
lighting of one specific classroom.

---

## HSV thresholds

OpenCV HSV ranges are **H: 0–179, S: 0–255, V: 0–255** — hue is halved to fit a byte,
which is the first thing to trip over when porting values from any other tool.

### Yellow — lane boundaries

| Channel | Range |
|---|---|
| Hue | 23 – 65 |
| Saturation | 40 – 255 |
| Value | 117 – 255 |

The hue band is wide because the painted yellow shifts toward green under fluorescent
light and toward orange near the windows. The real discrimination comes from the
saturation floor of 40: the track surface and its reflections are essentially
desaturated, so anything genuinely colored is lane paint. The value floor of 117
discards the same hue sitting in shadow.

### White — dashed highway markings

| Channel | Range |
|---|---|
| Hue | 0 – 179 (unrestricted) |
| Saturation | 0 – 50 |
| Value | 208 – 255 |

White has no meaningful hue, so hue is left fully open and the mask becomes "almost
colorless **and** very bright". The value floor of 208 is aggressive, and it has to
be — the track's own reflections under overhead lighting sit just below it. Lowering
this threshold was the single fastest way to fill the frame with phantom dashes.

### Red — parking square

| Channel | Range |
|---|---|
| Hue | 163 – 179 |
| Saturation | 83 – 255 |
| Value | 85 – 255 |

Red wraps around the hue circle (it occupies both ~0–10 and ~160–179). Only the
**upper** band is used here. That is a deliberate trade: the lower band overlaps
orange floor tones and skin, and losing some red sensitivity costs far less than a
false parking trigger, which ends the run outright. The low value floor of 85 keeps
the square detectable in its own shadow as the robot closes in.

---

## PD gains

| Parameter | Value | Applies to |
|---|---|---|
| `Kp` | 0.004 | Yellow lane mode |
| `Kd` | 0.006 | Yellow lane mode |
| `Kp_WHITE` | 0.003 | White highway mode |
| `Kd_WHITE` | 0.005 | White highway mode |
| `Kp_RED` | 0.005 | Red-seeking and parking |
| `MAX_ANGULAR` | 1.5 rad/s | Global clamp |

Gains are small because the error is measured in **pixels**, not meters. A 100 px
offset at `Kp = 0.004` yields 0.4 rad/s — a firm but not violent correction.

**Why `Kd > Kp`.** Unusual for a PD loop, and it follows from the pixel error being
large in magnitude but noisy. The proportional term must stay gentle to avoid
oscillation on curves; the derivative term does the real work of damping, and it can
be aggressive *only because* the error is smoothed over 3 frames first. Without that
smoothing this ratio oscillates badly.

**Why white gains are lower.** Dash clusters are intrinsically noisier than
continuous lines — the centroid shifts as dashes enter and leave the frame even when
the robot is dead centered. Softer gains stop the robot chasing that motion.

Higher proportional gains oscillated through curves; lower ones responded too late
and drifted wide. 0.004 was the point where neither happened.

---

## Speeds

| Parameter | Value | Phase |
|---|---|---|
| `MAX_SPEED` | 0.10 m/s | Straight yellow lane |
| `MIN_SPEED` | 0.04 m/s | Sharp curves |
| `WHITE_SPEED` | 0.10 m/s | Highway (fixed, no adaptation) |
| `SEEK_RED_SPEED` | 0.07 m/s | Searching for the square |
| `PARK_APPROACH_SPEED` | 0.04 m/s | Final approach |
| `PARK_FINAL_PUSH_SPEED` | 0.05 m/s | Blind seating push |

Conservative on purpose. Scoring rewarded clean completion over pace, so the target
was zero line crossings rather than minimum lap time — 4:11.22 with zero crossings
beat a faster run with a penalty.

Yellow-lane speed is the only adaptive one:

```python
speed_factor = min(1.0, abs_error / 150.0)
linear = MAX_SPEED - (MAX_SPEED - MIN_SPEED) * speed_factor
```

At 150 px of error the robot is at full `MIN_SPEED`. The divisor is roughly a quarter
of frame width — the error magnitude that in practice meant "real curve" rather than
"detection noise".

---

## Detection thresholds

| Parameter | Value | Purpose |
|---|---|---|
| `ROI_TOP_RATIO` | 0.55 | Process the bottom 45% of the frame only |
| `MIN_PIXELS` | 200 | Minimum blob area for a yellow line |
| `MIN_WHITE_PIXELS` | 80 | Minimum blob area for one white dash |
| `MIN_RED_PIXELS` | 500 | Minimum blob area to consider red at all |
| `MIN_LINE_SEPARATION` | 200 | Minimum px between two accepted yellow lines |
| `DEADBAND` | 10 | Error below this commands zero steering |
| `LANE_OFFSET` | 160 | Inferred lane center offset from a single line |

`MIN_WHITE_PIXELS` (80) is far below `MIN_PIXELS` (200) because a single dash is a
small object, and a distant one is smaller still. This threshold trades directly
against the value floor of the white mask: the aggressive V ≥ 208 is what makes such
a permissive area threshold safe.

`MIN_LINE_SEPARATION` (200) is the curve-splitting guard described in
[ARCHITECTURE.md](ARCHITECTURE.md#yellow--paired-line-detection). It must exceed the
width of one fragmented curved line but stay below the true lane width in pixels.

`LANE_OFFSET` (160) is half the expected lane width at the ROI. It is valid only
because the camera pitch is fixed — every parameter measured in pixels is implicitly
a function of camera geometry, and re-aiming the camera invalidates this one first.

---

## Smoothing and hysteresis

| Parameter | Value | Purpose |
|---|---|---|
| `TARGET_SMOOTH_WIN` | 4 frames | Moving average on the steering target |
| `ERROR_SMOOTH_WIN` | 3 frames | Moving average on error, feeds the D term |
| `MIN_DASH_BLOBS` | 2 | Dashes needed to enter highway mode |
| `WHITE_LOCK_FRAMES` | 8 | Hysteresis holding white mode |
| `T_DETECT_FRAMES` | 25 | Empty frames that declare a T-junction |
| `MIN_WHITE_FRAMES_TO_UNLOCK` | 15 | Highway frames before final stages unlock |
| `UNLOCK_COOLDOWN_FRAMES` | 10 | Grace period on leaving white mode |

At 20 Hz, a 4-frame window is 200 ms of lag — enough to absorb a dash flickering out,
short enough not to delay a genuine curve response. Longer windows visibly cut
corners.

`T_DETECT_FRAMES = 25` is 1.25 s of completely empty road. Long enough that no
transient occlusion reaches it, short enough that the robot has not driven far past
the junction by the time it fires.

---

## Turn parameters

| Parameter | Value |
|---|---|
| `TURN_PHASE_A_DURATION` | 1.0 s |
| `TURN_PHASE_A_LINEAR` | 0.05 m/s |
| `TURN_PHASE_A_ANGULAR` | 0.5 rad/s |
| `TURN_PHASE_B_DURATION` | 3.0 s (timeout) |
| `TURN_PHASE_B_LINEAR` | 0.06 m/s |
| `TURN_PHASE_B_BASE_ANGULAR` | 0.35 rad/s |
| `YELLOW_SAFETY_DIST` | 100 px |
| `Kp_TURN_AVOID` | 0.012 |
| `TURN_END_MIN_LINE_SEP` | 250 px |

Phase A at 0.5 rad/s for 1.0 s sweeps roughly 29°, enough to commit into the turn
while lines are still absent. Phase B's lower base rate (0.35 rad/s) leaves headroom
for the avoidance correction to add on top without hitting the clamp.

`Kp_TURN_AVOID` (0.012) is three times the lane-following `Kp`. Crossing the line
during the turn is a scored failure, so this loop is intentionally the most
aggressive one in the system.

`TURN_END_MIN_LINE_SEP` (250) exceeds `MIN_LINE_SEPARATION` (200) — declaring the
turn complete demands stronger evidence than ordinary lane tracking, because exiting
the turn early leaves the robot mid-junction with no way back into the state.

---

## Parking thresholds

| Parameter | Value | Meaning |
|---|---|---|
| `RED_AREA_PARKED` | 8000 px | Close enough to switch `SEEK_RED` → `PARK` |
| `RED_AREA_DONE` | 35000 px | Square fills the frame; stop |
| `RED_CY_DONE_RATIO` | 0.90 | Centroid at 90% frame height; stop |
| `RED_LOST_TIMEOUT` | 2.5 s | Declared but unused (see below) |
| `FINAL_PUSH_DURATION` | 1.5 s | Blind forward seating push |

Blob area serves as the distance proxy — it grows roughly with the inverse square of
range, so these thresholds are effectively distance gates.

The 4.4× gap between 8000 and 35000 gives the `PARK` state a long, slow approach
rather than a late scramble.

> **Known dead parameter:** `RED_LOST_TIMEOUT` is defined but never read. The
> original design waited 2.5 s to confirm the square was really gone; testing showed
> that losing red is an immediate, reliable success signal, so `PARK` now transitions
> to `FINAL_PUSH` on the first frame without red. The constant is left in place
> rather than silently removed, since it documents the abandoned approach.

---

## Re-tuning for a different track

In rough order of how quickly they break:

1. **HSV value floors** — the first thing to fail under new lighting. Run the node,
   watch the `YELLOW MASK` / `WHITE MASK` windows, and raise the floors until noise
   disappears.
2. **Pixel geometry** (`LANE_OFFSET`, `MIN_LINE_SEPARATION`, `YELLOW_SAFETY_DIST`) —
   invalid the moment the camera pitch or lane width changes.
3. **Area thresholds** — scale with camera resolution and mounting height.
4. **PD gains** — the most portable of the four; retune only after detection is clean.

Tuning gains before the masks are clean wastes time. Almost every symptom that looked
like a control problem turned out to be a detection problem.
