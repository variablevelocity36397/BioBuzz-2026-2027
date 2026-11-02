# Intake notes — how far and how long to run the intake

Photos (in `docs/photos/`):
- `huskylens-camera-view.png` — what the HuskyLens sees from its mount on the robot.
- `robot-holding-pollen-topdown.png` — top-down view of the StarterBot holding one pollen.

## What the photos show
1. **The bottom of the HuskyLens picture is blocked by the robot's own front plate** (the perforated
   goBILDA plate fills roughly the bottom 15–20% of the image). So a pollen ball **drops out of view
   before it reaches the intake**. The code must finish the pickup "blind" for the last few inches.
   → Use the moment the ball was last seen close (big box / low on screen) as the trigger, not "box is huge".
2. **The ball ends up in the middle channel**, about half a robot length (~6–8 in) behind the front edge.
   So after the ball first touches the Gecko wheels, the robot needs to keep moving forward about
   **6–8 in more**, with the intake on, for the ball to be fully in.
3. A **black wheel** is lying on the field. It's not yellow, so Color Recognition (pollen = ID 1) should ignore it,
   but keep the size/shape/height filters on.

## Recommended intake behaviour
- **Turn the intake ON early**: when the ball's box is ≥ ~45 px wide or its center is low (y ≥ ~170),
  i.e. roughly 15–20 in away — before it disappears under the camera.
- **Drive by DISTANCE, not just time**, after the ball is lost from view while close:
  drive forward **~12 in** (≈ 4–6 in until it reaches the rollers + 6–8 in to pull it into the channel)
  using the Pinpoint (or drive encoders), then **stop and keep the intake running ~0.5 s** more.
- **Safety timeout ~2 s** on that forward creep, in case the robot gets stuck.
- **Slow creep**: ~0.3 power × SPEED (0.5) ≈ 0.15 motor power. On the StarterBot drive
  (312 rpm Yellow Jacket, 104 mm wheels, ~67 in/s free speed) that's very roughly 7–10 in/s,
  so 12 in takes about 1.2–1.7 s. Measure it on the real robot and adjust.

## Numbers to measure on the real robot (please fill in)
- Distance from the camera lens to where the ball first touches the intake rollers: ____ in
- How far the robot must drive after touching for the ball to sit in the channel: ____ in
- HuskyLens box width / y when the ball is just about to disappear under the camera: w ____  y ____
- Intake run time that reliably pulls a ball all the way in from touching: ____ s

## Status of the code (as of this note)
- `Autos/PollenFollower.java` now **finds, chases and intakes** pollen: spins to search when no ball is seen,
  locks onto one ball (`LOCK_JUMP_PX = 70`), turns the intake on in the intake zone (`INTAKE_ZONE_WIDTH = 45` /
  `INTAKE_ZONE_Y = 170`), and when the ball vanishes while close (or reaches `CLOSE_WIDTH = 90`) it creeps
  **`GRAB_DISTANCE = 12` in by Pinpoint** at `GRAB_FORWARD = 0.3`, with a `GRAB_TIMEOUT = 2.0` s safety limit, then keeps
  the intake on `INTAKE_SETTLE = 0.5` s. Telemetry "Last grab" shows the distance and time each grab actually drove.
- `Autos/PollenCollector.java` has the time-based intake (`INTAKE_TIME = 2.0` s, `GRAB_CREEP = 0.3`,
  intake zone `INTAKE_ZONE_WIDTH = 45` / `INTAKE_ZONE_Y = 170`, `CLOSE_WIDTH = 90`, and it finishes the grab
  if the ball vanishes while close). Next step suggested: switch the creep to a distance (`~12 in`) using the
  Pinpoint, with the 2 s timeout kept as a safety limit.
