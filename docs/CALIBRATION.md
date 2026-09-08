# Camera Setup & Calibration

Three bugs in the stock calibration pipeline, and one hardware decision that mattered
more than any of them.

## The camera angle

The camera started aimed far down the track — roughly 20 m of visible road.

That sounds like an advantage and is not. Almost all of that distance is compressed
into the top few rows of the image by perspective, where lane markings are a handful
of pixels wide, blur together, and produce centroids that jitter frame to frame. The
robot was steering on noise.

Pitching the camera **downward** cut the visible range to about 10 m and fixed the
lane following. The nearby road now occupies most of the frame, marking edges are
sharp, and centroids are stable. Detection consistency improved and steering
oscillation dropped — without touching a single gain.

The lesson generalizes: for a reactive controller, **the useful signal is close, not
far**. Range only helps a planner that can act on it.

> Every pixel-denominated parameter in [TUNING.md](TUNING.md) — `LANE_OFFSET`,
> `MIN_LINE_SEPARATION`, `YELLOW_SAFETY_DIST`, all the area thresholds — is a
> function of this camera pose. Re-aiming the camera invalidates them.

---

## Three bugs in the stock calibration pipeline

Intrinsic calibration would not run. The launch file shipped with the course
materials had three independent problems, and each masked the next.

### 1 — A republish node with nothing to republish

The pipeline ran an `image_transport republish` node to decompress a compressed
camera stream. But the `cv_camera` driver on our setup publishes **raw** images
directly — the compressed topics it was subscribing to never existed.

The node did not error. It sat waiting forever, and the pipeline downstream of it
simply never received frames.

**Fix:** remove the republish node entirely.

### 2 — A full topic path where a namespace was expected

```bash
# Original — wrong
camera:=/camera/camera_info

# Fixed
camera:=/camera/rgb
```

`cameracalibrator.py` takes a **namespace** and appends the topic names itself,
resolving `<ns>/image_raw` and `<ns>/camera_info`. Handing it a full topic path made
it look for `/camera/camera_info/image_raw`, which does not exist.

**Fix:** pass the namespace prefix.

### 3 — Rectifying images before calibrating

The pipeline ran `image_proc` ahead of the calibration tool, feeding it **rectified**
images.

This one is a logic error rather than a plumbing error, and it is the dangerous kind
because it fails quietly. Calibration exists to *estimate* lens distortion, so it
needs the raw distorted image — the distortion is the signal. Rectified input has
already had distortion removed (using whatever parameters were loaded), so the tool
converges happily and reports a calibration that is meaningless.

**Fix:** remove `image_proc` from the calibration path so the tool receives the raw
distorted stream. Rectification belongs *after* calibration, never before.

---

## Why these are worth writing down

None of the three announced itself. The republish node blocked in silence, the
namespace error produced a tool that waited on a topic nobody was publishing, and the
rectification bug produced *plausible numbers that were wrong* — the worst outcome of
the three, since it would have propagated into every downstream computation without
ever looking broken.

Debugging came down to inspecting the actual topic graph — `rostopic list`,
`rostopic hz`, `rqt_graph` — instead of trusting the launch file to describe what was
running. A launch file states intent; the topic graph states fact, and on this
pipeline they disagreed in three places.
