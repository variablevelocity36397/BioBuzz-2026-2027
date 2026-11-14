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

## Driver Hub config — NEW: "BioBuzz Robot" (Nov 11 2026)
All config names live in **`Autos/RobotNames.java`**. Each part has a list of names; the code uses the
first one that exists, so both "BioBuzz Robot" and the old "Starter Bot" config work. Add new names there.
| Hub | Port | Name | Used by pollen code? |
|---|---|---|---|
| Control Hub motor | 0 / 1 | `blwheel` / `flwheel` | yes (drive) |
| motor | 2 | `transfer` | **yes** - runs WITH the intake (Nov 11, user asked), direction `TRANSFER_DIR` |
| motor | 3 | `intake2` | yes (intake, direction `INTAKE2_DIR`) |
| I2C | 1 | `husky lens` (with a space) | yes |
| Expansion Hub servo | 0 / 1 / 2 | `turretleft` / `adjustableintake` / `hood` | no (position servos) |
| ? | ? | front right / back right wheels — expected `frwheel` / `brwheel` | **not confirmed yet** |
| ? | ? | Pinpoint — expected `pinpoint` (only Pollen Collector uses it) | **not confirmed yet** |
Old "Starter Bot" names (`fl fr bl br intakemotor leftintake rightintake husylens`) are still in the lists.

Motor/intake directions are shared constants in `Autos/PollenCollector.java` (`FL_DIR`, `LEFT_INTAKE_DIR`, ...).

## OpModes
- **`Autos/PollenFollower.java` — "Pollen Follower"** (Nov 11: `COLLECT = true` - chase + intake) **`SWEEP = true`** (Nov 11): user wants: drive to ball (intake off), when close intake ON, roll 6 in forward, stop 0.5 s with intake on, intake off, then search for the next ball. No back-up/check. `SWEEP = false` = old pass/back-up/check/retry. Also: SPEED 0.7, SEARCH_TURN 0.25, MAX_TURN 0.45, KP_TURN 0.005, APPROACH_POWER 0.65, 0.25 s stop when a ball is first spotted (skipped while sweeping). Intake dirs: INTAKE_MOTOR_DIR and INTAKE2_DIR both REVERSE (user said it was out-taking). (the one the user is testing now).
  Follows AND intakes (`COLLECT = true`): turns to face the pollen (`TRACK_MODE = TURN`), intake on when close,
  drives `PASS_DISTANCE_IN` (12 in, wheel encoders) past where the ball vanished, backs up to check, retries if missed.
  `COLLECT = false` = follow only (holds `FOLLOW_WIDTH` distance, no intake). See docs/INTAKE_NOTES.md.
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

## Intake — how far / how long (READ THIS for intake work)
See **`docs/INTAKE_NOTES.md`** and the photos in **`docs/photos/`**
(`huskylens-camera-view.png` = what the HuskyLens sees from the robot; `robot-holding-pollen-topdown.png`
= the robot holding a ball). Short version: the ball disappears under the camera before reaching the intake,
so turn the intake on early and finish the pickup blind; drive ~12 in forward after it vanishes
(Pinpoint distance, 2 s safety timeout), then keep the intake on ~0.5 s. Measure and fill in the numbers in the notes.
