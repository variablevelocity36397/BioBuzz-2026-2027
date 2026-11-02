# CLAUDE.md — BioBuzz-2026-2027 (FTC robot code)

Context handed over from the Claude chat that set up this project. Read this before changing anything.

## What this project is
FTC 2026-27 **BIOBUZZ** robot code for a goBILDA **StarterBot** (mecanum) on a REV Control Hub.
Android Studio project; team code lives in `TeamCode/`. Build/deploy with ▶ Run in Android Studio.

## Rules
- **Never edit the SDK module** `FtcRobotController/` (especially `FtcRobotControllerActivity.java`). It was
  once overwritten by pasted robot code and broke the build; it was restored from git.
- Team code goes in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
- Don't paste old copies of files over these — older copies were written for a different project
  (`pedropathingquickstart`) and import `org.firstinspires.ftc.teamcode.pedro.Constants`, which doesn't exist here.
- Explain things simply; the user is a student.

## Driver Hub config ("Starter Bot") — names the code uses
| Device | Name | Notes |
|---|---|---|
| Front left / front right / back left / back right motors | `fl`, `fr`, `bl`, `br` | mecanum; `bl` is port 3 on the hub with `fl` and `shooter` |
| Intake motor | `intakemotor` | |
| Intake CR servos | `leftintake`, `rightintake` | servo ports 0 and 4 |
| Other | `shooter` (motor), `windmill` (CR servo) | not used by the pollen code yet |
| Odometry | `pinpoint` | goBILDA Pinpoint, I2C |
| Camera | `husylens` | HuskyLens, I2C. **Spelled without the k** to match the config. Color Recognition, pollen = ID 1 |

Motor/intake directions are shared constants in `Autos/PollenCollector.java` (`FL_DIR`, `LEFT_INTAKE_DIR`, ...).

## OpModes
- **`Autos/PollenFollower.java` — "Pollen Follower"** (the one the user is testing now).
  Follow-only, **no intake**. Turns to face the pollen (`TRACK_MODE = TURN`; `STRAFE` / `BOTH` available),
  drives forward/back to hold distance (`FOLLOW_WIDTH` = 60 px ≈ 15 in), stops instantly when the ball is lost.
  Tuning: `KP_TURN`, `TURN_SIGN`, `STRAFE_SIGN`, `FORWARD_SIGN`, `SPEED` (0.5).
- **`Autos/PollenCollector.java` — "Pollen Collector (P)"**: drives to field coordinates with a P controller
  (Pinpoint), slows down within `SLOW_DOWN_DIST` (24 in), searches for pollen, intakes it, goes to SCORE.
  `START`, `SEARCH_SPOTS`, `SCORE` are **placeholders**. Pinpoint pod offsets are 0 until the Pinpoint tuner is run.
  Uses Marrow Prompter (INIT settings), TimerEx (match clock), Zones (ignore balls estimated off-field),
  `@BulkCaching`. Camera mount constants (`CAMERA_HEIGHT_IN` etc.) need measuring.
- **`TestComponents/StarterBotDirectionTest.java`**: X/Y/A/B spin fl/fr/bl/br, D-pad drives/turns,
  bumpers/triggers run intake parts. Use it to find wrong wheel names/directions.

## Libraries
- Marrow `com.skeletonarmyftc.marrow:core:1.2.2` (in `TeamCode/build.gradle`). Used: `prompts.*`, `TimerEx`,
  `zones.Point/PolygonZone/Zone`, `BulkCaching` (annotation element is `mode`, default AUTO — use bare `@BulkCaching`).
- Pedro Pathing revhub 3.0.0 (`PinpointLocalizer`, `PinpointConfig`, `Pose`, `PoseFactory`).

## Known issues / history
- Earlier on the robot: it chased non-pollen (fixed with size/shape/height filters + 3-frame confirm),
  drove forward blindly after losing the ball (removed), and once "strafed" when it should have turned —
  likely a wheel name/direction mix-up; check with StarterBotDirectionTest if it happens again.
- HuskyLens can only see pollen within roughly 2–3 ft.

## Simulator
`~/StudioProjects/virtual_robot` (Beta8397 virtual_robot, JDK = liberica-full-17). OpModes
"Pollen Follower SIM" and "Pollen Collector SIM" in `TeamCode/src/org/firstinspires/ftc/teamcode/`.
Fake pollen, no real camera. Before each run re-pick "Mecanum Bot" so the robot resets to the center.
