# Images

## Diagrams

| File | Shows |
|---|---|
| `pipeline.svg` | Vision pipeline from camera frame to `/cmd_vel` |
| `state_machine.svg` | The seven mission states and their transition conditions |

Both are hand-authored SVG, so they stay sharp at any zoom and are diffable in git.
State box colors match the live `DECISION` debug window.

## Screenshots still worth adding

These have to come from the robot or the run recordings:

| File | What to capture |
|---|---|
| `decision_view.png` | The `DECISION` window mid-run — detected lines, target, telemetry panel |
| `masks.png` | Yellow / white / red masks beside the source frame |
| `turn_safety.png` | The `TURN_LEFT` safety corridor with a yellow line encroaching |
| `track.jpg` | The physical track |

Good sources for stills:
[robot run](https://youtu.be/nZOmZpWM95w) · [screen capture](https://youtu.be/SDK73mEr38g)

Embed with:

```markdown
![Decision view](docs/images/decision_view.png)
```
