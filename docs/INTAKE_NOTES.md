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
- `Autos/PollenFollower.java`: see "Done" below (merged version: spin search + one-ball lock + drive past + check/retry).
- `Autos/PollenCollector.java` has the time-based intake (`INTAKE_TIME = 2.0` s, `GRAB_CREEP = 0.3`,
  intake zone `INTAKE_ZONE_WIDTH = 45` / `INTAKE_ZONE_Y = 170`, `CLOSE_WIDTH = 90`, and it finishes the grab
  if the ball vanishes while close). Next step suggested: switch the creep to a distance (`~12 in`) using the
  Pinpoint, with the 2 s timeout kept as a safety limit.

## Test video, Nov 1 2026 (frames: `docs/photos/test-video-1-first-16s.jpg`, `test-video-2-next-16s.jpg`)
- The follow-only version turned to face the ball and drove up to it, then **stopped at the ball and nudged it**.
  The intake never ran (that version had no intake on purpose).
- User wants: intake ON when close, **drive a little PAST the ball** so it's surely in, then **check** it's in,
  and if not, **try again another way**.

. No ball in view -> SPIN toward the side it was last seen (`SEARCH_TURN` = 0.35, flip `SEARCH_SIGN` if backwards),
   after a `LOST_GRACE` = 0.4 s pause.
1. LOCK ON to one ball (ignores boxes more than `LOCK_JUMP_PX` = 70 px from it), turn to face it and drive at it.
2. Close (box w >= 45 or y >= 170) -> intake ON, keep driving at it (no more stopping in front).
3. Ball vanishes under the camera while close -> drive `PASS_DISTANCE_IN` = 12 in past (wheel encoders,
   312 rpm / 104 mm -> 0.0239 in/tick), keep intake on `INTAKE_EXTRA_TIME` = 0.5 s, 2.5 s safety timeout.
4. Back up `BACKUP_IN` = 6 in and look for 0.6 s. A ball with box w >= `MISSED_WIDTH` (35) still in front = missed ->
   strafe 3 in sideways (alternating sides) and re-approach with intake on; up to `MAX_RETRIES` = 2.
Telemetry shows Collected / missed / retries and "Wheels moved" inches. Tune PASS_DISTANCE_IN first.
