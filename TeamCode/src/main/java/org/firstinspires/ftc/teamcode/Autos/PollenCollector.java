package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.skeletonarmy.marrow.BulkCaching;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.MessagePrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.Zone;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

/*
  Pollen Collector - P control to field coordinates + HUSKYLENS + Marrow
  ---------------------------------------------------------------------
  Marrow features used:
    - Prompter  : pre-match settings on the Driver Hub during INIT (gamepad D-pad / A / B)
    - TimerEx   : match clock, heads to SCORE before auto ends
    - Zones     : ignores "pollen" whose estimated field spot is OFF the field or in a no-go area
                  (stops it chasing people, bags and reflections outside the walls)
    - @BulkCaching : faster hardware reads every loop

  HUSKYLENS SETUP:
    - Protocol Type = I2C, Color Recognition mode, pollen color learned as ID 1
    - Robot config name: "husylens" (matches your Driver Hub spelling)

  The Pinpoint tracks where the robot is on the field (x, y in inches, heading).
  A P controller drives the mecanum base to each target coordinate:

      power = kP * error        (error = target - where we are now)

  So the robot drives fast when it's far away and slows down as it gets close.

  The routine:
    1. Drive (P control) to each SEARCH SPOT coordinate
    2. At each spot, look for pollen with the HuskyLens and P-steer onto it
    3. Intake it, then go to the next spot
    4. When it has enough pollen or time is almost up, drive to the SCORE coordinate

  Coordinates use the same field as your Pedro autos (0-144 inches, heading in degrees).
  The ones below are PLACEHOLDERS - measure your real BIOBUZZ spots and change them.
*/

@BulkCaching   // Marrow: automatic bulk reads (AUTO mode) for this OpMode
@Autonomous(name = "Pollen Collector (P)", group = "Autonomous")
public class PollenCollector extends LinearOpMode {

    // ---------- Field coordinates (CHANGE THESE) ----------
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose START = p.of(72, 9, 90);       // where the robot starts
    private final Pose[] SEARCH_SPOTS = {             // where to look for pollen
            p.of(48, 48, 90),
            p.of(72, 60, 90),
            p.of(96, 48, 90),
    };
    private final Pose SCORE = p.of(72, 100, 90);     // where to go when done

    // ---------- Pinpoint (odometry) for THIS robot ----------
    // Run the Pinpoint tuner (Localization/Tuning.java) on this robot and copy its pod offsets
    // and directions here. Until then, positions may drift or be mirrored.
    public static final PinpointConfig PINPOINT = new PinpointConfig(c -> {
        c.name.set("pinpoint");                                     // Driver Hub config name
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0);                                      // TODO: from the tuner (inches)
        c.yPodOffset.set(0.0);                                      // TODO: from the tuner (inches)
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // ---------- Directions (shared with "StarterBot Direction Test") ----------
    // If a wheel or intake part spins the wrong way in the test, flip it HERE (FORWARD <-> REVERSE).
    public static final DcMotorSimple.Direction FL_DIR = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BL_DIR = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FR_DIR = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BR_DIR = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction INTAKE_MOTOR_DIR = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction LEFT_INTAKE_DIR  = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_INTAKE_DIR = DcMotorSimple.Direction.REVERSE;

    // ---------- Overall speed ----------
    // Multiplies EVERY drive power. 1.0 = full speed, 0.5 = half speed. Lower = slower and gentler.
    private static final double SPEED = 0.5;

    // ---------- P gains for driving to coordinates ----------
    private static final double KP_DRIVE    = 0.06;  // forward/back power per inch of error
    private static final double KP_STRAFE   = 0.08;  // sideways power per inch (strafing needs more)
    private static final double KP_TURN     = 1.2;   // turn power per radian of heading error
    private static final double MAX_POWER   = 0.6;   // speed limit while driving to a point
    private static final double POS_TOL     = 1.0;   // "arrived" when within this many inches...
    private static final double HEAD_TOL    = Math.toRadians(3); // ...and this many degrees
    private static final double DRIVE_TIMEOUT = 8.0; // give up on a point after this many seconds
    private static final double SLOW_DOWN_DIST  = 24.0; // start slowing down this many inches from the target
    private static final double MIN_DRIVE_POWER = 0.25; // speed limit right at the target (before SPEED)

    // ---------- HuskyLens ----------
    private static final int    POLLEN_ID       = 1;     // ID the HuskyLens learned for pollen
    private static final int    SCREEN_W        = 320;   // HuskyLens image size
    private static final int    SCREEN_H        = 240;
    private static final double HFOV_DEG        = 60.0;  // HuskyLens field of view (approx.)
    private static final double VFOV_DEG        = 45.0;

    // Where the HuskyLens is on the robot (MEASURE THESE - used to estimate where the ball is on the field)
    private static final double CAMERA_HEIGHT_IN       = 8.0;  // lens height above the floor
    private static final double CAMERA_PITCH_DOWN_DEG  = 25.0; // how far the camera is tilted DOWN
    private static final double CAMERA_FORWARD_IN      = 7.0;  // lens distance in front of the robot's center
    private static final double BALL_RADIUS_IN         = 1.5;  // pollen is ~3 in across

    // ---------- P gain for steering onto pollen with the camera ----------
    private static final double KP_VISION_TURN  = 0.004; // turn power per PIXEL the ball is off-center
    private static final double APPROACH_POWER  = 0.4;   // forward power when the ball is far (before SPEED)
    private static final double MIN_APPROACH    = 0.24;  // slowest forward power when the ball is close
    private static final double INTAKE_START    = 0.5;   // (internal) remembered closeness that counts as "at the intake"

    // ---------- INTAKE ZONE: pollen this close -> intake ON and drive forward to get it ----------
    // A ball counts as "in the intake zone" if EITHER is true (watch the "Ball box" telemetry line to tune):
    private static final int    INTAKE_ZONE_WIDTH = 45;  // its box is at least this many pixels wide, OR
    private static final int    INTAKE_ZONE_Y     = 170; // its center is this low on the screen (0 top .. 240 bottom)
    private static final double ZONE_FORWARD      = 0.3; // forward power while driving onto the ball (before SPEED)
    private static final int    CLOSE_WIDTH       = 90;  // box this wide = ball is right at the intake -> grab
    private static final boolean SEARCH_MOVES   = false; // false = stand still when no pollen is seen
    // true  = sweep left/right and creep forward
    private static final double SEARCH_TURN     = 0.2;   // turn power while sweeping left/right
    private static final double CREEP_POWER     = 0.2;   // forward power while creeping toward far pollen
    private static final double MAX_CREEP       = 24.0;  // inches it may creep from the search spot (~camera range)
    private static final double SEARCH_TIMEOUT  = 9.0;   // seconds to look at one spot (3 sweep cycles)

    // ---------- Filters so it only chases real pollen (tune these!) ----------
    private static final int    MIN_BOX_PX      = 10;    // ignore tiny boxes (noise, far-away specks)
    private static final double MIN_ASPECT      = 0.6;   // a ball's box is about square:
    private static final double MAX_ASPECT      = 1.6;   //   width / height must be between these
    private static final int    MIN_Y           = 60;    // ignore boxes near the TOP of the screen (walls, people).
    //   0 = top, 240 = bottom
    private static final int    CONFIRM_FRAMES  = 3;     // must see it this many frames in a row before chasing

    // ---------- Marrow Zones: only chase pollen whose estimated field spot is allowed ----------
    private static final boolean USE_FIELD_ZONES = true;   // false = skip this check
    // The field, shrunk 2 in from the walls (field is 144 x 144 in)
    private final Zone FIELD = new PolygonZone(
            new Point(2, 2), new Point(142, 2), new Point(142, 142), new Point(2, 142));
    // Areas to never chase pollen in (e.g. the HIVE, the other alliance's side). Add your own:
    //   new CircleZone(new Point(72, 72), 12)    or    new PolygonZone(new Point(x1,y1), new Point(x2,y2), ...)
    private final Zone[] NO_GO_ZONES = { };

    // ---------- Intake ----------
    private static final double INTAKE_TIME = 2.0;   // seconds to keep intaking while creeping forward
    private static final double GRAB_CREEP  = 0.3;   // forward power while creeping over the ball (before SPEED)

    // ---------- Match time ----------
    private static final double AUTO_LENGTH  = 30.0; // check the BIOBUZZ manual
    private static final double RETURN_TIME  = 5.0;  // head to SCORE when this many seconds are left

    // ---------- Marrow ----------
    private final Prompter prompter = new Prompter(this);
    private final TimerEx matchTimer = new TimerEx(AUTO_LENGTH);
    private int     maxPollen  = 4;
    private double  startDelay = 0.0;
    private boolean useTimer   = true;

    // ---------- Hardware ----------
    private PinpointLocalizer localizer;
    private HuskyLens huskyLens;
    private DcMotor flwheel, frwheel, blwheel, brwheel, intakemotor;
    private CRServo leftintake, rightintake;

    private int pollenCollected = 0;
    private String status = "Init";

    @Override
    public void runOpMode() {
        // Names match the "Starter Bot" config on the Driver Hub
        flwheel = hardwareMap.get(DcMotor.class, "fl");
        frwheel = hardwareMap.get(DcMotor.class, "fr");
        blwheel = hardwareMap.get(DcMotor.class, "bl");
        brwheel = hardwareMap.get(DcMotor.class, "br");
        flwheel.setDirection(FL_DIR);
        blwheel.setDirection(BL_DIR);
        frwheel.setDirection(FR_DIR);
        brwheel.setDirection(BR_DIR);
        for (DcMotor m : new DcMotor[]{flwheel, frwheel, blwheel, brwheel}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        leftintake  = hardwareMap.get(CRServo.class, "leftintake");
        rightintake = hardwareMap.get(CRServo.class, "rightintake");
        // The two Gecko wheels face each other, so one spins the opposite way.
        // If a wheel pushes pollen OUT, swap which one is REVERSE.
        intakemotor.setDirection(INTAKE_MOTOR_DIR);
        leftintake.setDirection(LEFT_INTAKE_DIR);
        rightintake.setDirection(RIGHT_INTAKE_DIR);

        // Pinpoint, using the PINPOINT settings above
        localizer = new PinpointLocalizer(hardwareMap, PINPOINT);
        localizer.setPose(START);

        huskyLens = hardwareMap.get(HuskyLens.class, "husylens");   // matches the Driver Hub config spelling
        boolean cameraFound = huskyLens.knock();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        // ---------- Pre-match settings (Marrow Prompter) ----------
        prompter.prompt("max_pollen", new ValuePrompt<>("How many pollen to collect?", Integer.class, 1, 4, 4, 1))
                .label("Max pollen")
                .prompt("delay", new ValuePrompt<>("Start delay (seconds)", Double.class, 0.0, 10.0, 0.0, 0.5))
                .label("Start delay")
                .prompt("use_timer", new BooleanPrompt("Go to SCORE before auto ends?", true))
                .label("Use match timer")
                .prompt(new MessagePrompt(cameraFound
                        ? "HuskyLens ready. Place robot at the START coordinate!"
                        : "WARNING: HuskyLens NOT FOUND - check I2C wiring, Protocol = I2C, config name 'husylens'"))
                .showSummary()
                .onComplete(() -> {
                    maxPollen  = prompter.get("max_pollen");
                    startDelay = prompter.get("delay");
                    useTimer   = prompter.get("use_timer");
                });

        while (opModeInInit()) {
            prompter.run();
            localizer.update();
        }
        if (isStopRequested()) return;

        // ---------- Autonomous ----------
        matchTimer.start();
        waitSeconds(startDelay);

        for (Pose spot : SEARCH_SPOTS) {
            if (!opModeIsActive() || pollenCollected >= maxPollen || timeToReturn()) break;

            status = "Driving to search spot";
            driveTo(spot);

            // Keep grabbing pollen at this spot until none are left in view
            while (opModeIsActive() && pollenCollected < maxPollen && !timeToReturn()) {
                if (!findAndGrabPollen()) break;
            }
        }

        status = "Driving to SCORE";
        intakeOff();
        driveTo(SCORE);

        stopDrive();
        status = "Done - collected " + pollenCollected;
        showTelemetry(null);
        while (opModeIsActive()) {
            localizer.update();
            showTelemetry(null);
        }
    }

    // =====================================================================
    //  P CONTROLLER: drive to a field coordinate
    // =====================================================================
    private void driveTo(Pose target) {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT) {
            localizer.update();
            Pose now = localizer.pose();

            // 1. Error = target - current (field coordinates)
            double errX = target.x() - now.x();
            double errY = target.y() - now.y();
            double errH = angleWrap(target.heading() - now.heading());

            // 2. Done?
            if (Math.hypot(errX, errY) < POS_TOL && Math.abs(errH) < HEAD_TOL) break;

            // 3. Turn the field error into the robot's point of view
            double h = now.heading();
            double forwardErr =  errX * Math.cos(h) + errY * Math.sin(h);
            double leftErr    = -errX * Math.sin(h) + errY * Math.cos(h);

            // 4. P control: power = kP * error
            double forward = KP_DRIVE  * forwardErr;
            double left    = KP_STRAFE * leftErr;
            double turn    = KP_TURN   * errH;    // + = turn left (counter-clockwise)

            // 5. Slow down smoothly as we get close
            double[] slowed = slowNearTarget(forward, left, Math.hypot(errX, errY));
            setMecanum(slowed[0], slowed[1], turn, MAX_POWER);
            showTelemetry(target);
        }
        stopDrive();
    }

    // =====================================================================
    //  Vision: P-steer onto the closest pollen and intake it
    //  Returns false if no pollen was found at this spot.
    // =====================================================================
    private boolean findAndGrabPollen() {
        ElapsedTime searchTimer = new ElapsedTime();
        localizer.update();
        Pose searchStart = localizer.pose();   // where we started looking (limits the creep)
        lastCloseness = 0;

        while (opModeIsActive() && !timeToReturn()) {
            localizer.update();
            Blob pollen = findClosestPollen();

            if (pollen == null) {
                // 3. Lost the ball right in front of the intake? It went under the camera - grab it.
                if (lastCloseness >= INTAKE_START) {
                    lastCloseness = 0;
                    grabPollen();
                    return true;
                }
                if (searchTimer.seconds() > SEARCH_TIMEOUT) return false;  // nothing here
                status = "No pollen - stopped";
                intakeOff();
                if (SEARCH_MOVES) searchSweep(searchTimer.seconds(), searchStart);
                else stopDrive();
            } else if (pollen.w >= CLOSE_WIDTH) {
                lastCloseness = 0;
                grabPollen();
                return true;
            } else {
                // P control on the pixel error: ball to the right -> turn right
                status = "Driving to pollen (slowing down)";
                double turn = -KP_VISION_TURN * (pollen.x - SCREEN_W / 2.0);

                boolean inIntakeZone = pollen.w >= INTAKE_ZONE_WIDTH || pollen.y >= INTAKE_ZONE_Y;

                if (inIntakeZone) {
                    // Pollen is close to the intake: turn the intake ON and drive forward to get it
                    status = "IN INTAKE ZONE - intake on, driving to it";
                    intakeOn();
                    lastCloseness = 1.0;   // if it slides under the camera now, grabPollen() finishes the job
                    setMecanum(ZONE_FORWARD, 0, turn * 0.5, 1.0);   // gentle steering, mostly straight
                } else {
                    // Still far: slow down as it gets bigger (closer), intake off
                    double closeness = Range.clip((double) pollen.w / CLOSE_WIDTH, 0, 1);
                    lastCloseness = closeness;
                    double forward = MIN_APPROACH + (APPROACH_POWER - MIN_APPROACH) * (1.0 - closeness);
                    intakeOff();
                    setMecanum(forward, 0, turn, 1.0);
                }
                searchTimer.reset();
            }
            showTelemetry(null);
        }
        return false;
    }

    /**
     * Pollen far away can be too small for the camera to see, so instead of only spinning,
     * repeat a 3-second cycle: look left, look right, re-center, then creep forward a bit.
     * The creep stops once the robot is MAX_CREEP inches from where it started searching.
     */
    private void searchSweep(double t, Pose searchStart) {
        double phase = t % 3.0;
        Pose now = localizer.pose();
        double crept = Math.hypot(now.x() - searchStart.x(), now.y() - searchStart.y());

        if (phase < 0.6)       setMecanum(0, 0,  SEARCH_TURN, 1.0);   // look left
        else if (phase < 1.8)  setMecanum(0, 0, -SEARCH_TURN, 1.0);   // look right
        else if (phase < 2.4)  setMecanum(0, 0,  SEARCH_TURN, 1.0);   // back to center
        else if (crept < MAX_CREEP) {
            status = "Searching (creeping forward)";
            setMecanum(CREEP_POWER, 0, 0, 1.0);                       // move closer
        } else {
            stopDrive();                                              // crept far enough
        }
    }

    private void grabPollen() {
        status = "Intaking";
        intakeOn();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < INTAKE_TIME) {
            localizer.update();
            setMecanum(GRAB_CREEP, 0, 0, 1.0);   // slowly creep over the ball while intaking
            showTelemetry(null);
        }
        stopDrive();
        waitSeconds(0.3);                   // let the intake finish pulling it in
        intakeOff();
        pollenCollected++;
    }

    /** One pollen box from the HuskyLens (pixels: x,y = center, w,h = size; 0,0 = top-left). */
    private static class Blob {
        final int x, y, w, h;
        Point field;   // estimated spot on the field (null if it can't be estimated)
        Blob(int x, int y, int w, int h) { this.x = x; this.y = y; this.w = w; this.h = h; }
        /** degrees right of center */ double tx() { return (x - SCREEN_W / 2.0) / SCREEN_W * HFOV_DEG; }
        /** degrees above center   */ double ty() { return (SCREEN_H / 2.0 - y) / SCREEN_H * VFOV_DEG; }
    }

    private int seenFrames = 0;
    private double lastCloseness = 0;   // how close the ball was the last time we saw it (0 far .. 1 at intake)
    private int lastRaw = 0, lastRejected = 0;
    private Blob lastBlob = null;       // last pollen seen (for telemetry)
    private String lastReject = "";

    /** Biggest blob that LOOKS like pollen, and only after it has been seen CONFIRM_FRAMES in a row. */
    private Blob findClosestPollen() {
        List<Blob> blobs = readHuskyLens();

        Blob best = null;
        lastRaw = blobs.size();
        lastRejected = 0;
        for (Blob b : blobs) {
            String why = rejectReason(b);
            if (why != null) { lastRejected++; lastReject = why; continue; }
            if (best == null || b.w > best.w) best = b;
        }

        if (best == null) { seenFrames = 0; return null; }
        lastBlob = best;
        seenFrames++;
        return (seenFrames >= CONFIRM_FRAMES) ? best : null;
    }

    /** All boxes the HuskyLens sees with the pollen ID. */
    private List<Blob> readHuskyLens() {
        List<Blob> out = new ArrayList<>();
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks == null) return out;   // camera didn't answer this frame
        for (HuskyLens.Block b : blocks) {
            if (b.id == POLLEN_ID) out.add(new Blob(b.x, b.y, b.width, b.height));
        }
        return out;
    }

    /** null = looks like pollen. Otherwise a short reason it was ignored (shown on telemetry). */
    private String rejectReason(Blob b) {
        if (b.w < MIN_BOX_PX || b.h < MIN_BOX_PX) return "too small";
        double aspect = (double) b.w / b.h;
        if (aspect < MIN_ASPECT || aspect > MAX_ASPECT) return "not round";
        if (b.y < MIN_Y) return "too high (not on floor)";

        if (USE_FIELD_ZONES) {
            b.field = estimateFieldPoint(b);
            if (b.field != null) {
                if (!FIELD.contains(b.field)) return "off the field";
                for (Zone z : NO_GO_ZONES) if (z.contains(b.field)) return "in a no-go zone";
            }
        }
        return null;
    }

    /**
     * Where is this ball on the field? Uses the camera height/tilt to get the distance,
     * then the robot's Pinpoint position to place it on the field. Null if it's too close to the horizon.
     */
    private Point estimateFieldPoint(Blob b) {
        double downDeg = CAMERA_PITCH_DOWN_DEG - b.ty();   // angle below horizontal to the ball
        if (downDeg < 2) return null;                     // too flat to estimate (would be miles away)
        double dist = (CAMERA_HEIGHT_IN - BALL_RADIUS_IN) / Math.tan(Math.toRadians(downDeg));

        double fwd  = CAMERA_FORWARD_IN + dist;                    // in front of robot center
        double left = -dist * Math.tan(Math.toRadians(b.tx()));    // +tx = right, so left is negative

        Pose now = localizer.pose();
        double h = now.heading();
        double x = now.x() + fwd * Math.cos(h) - left * Math.sin(h);
        double y = now.y() + fwd * Math.sin(h) + left * Math.cos(h);
        return new Point(x, y);
    }

    // =====================================================================
    //  Helpers
    // =====================================================================

    /** forward, left (strafe), turn (+ = counter-clockwise). Same wheel math as your TeleOp. */
    private void setMecanum(double forward, double left, double turn, double maxPower) {
        double y  = Range.clip(forward, -maxPower, maxPower);
        double x  = Range.clip(-left,   -maxPower, maxPower);   // TeleOp uses + = right
        double rx = Range.clip(-turn,   -maxPower, maxPower);   // TeleOp uses + = clockwise

        double lf = y + x + rx;
        double lb = y - x + rx;
        double rf = y - x - rx;
        double rb = y + x - rx;

        // Scale down if any wheel is over 1 so the robot keeps its direction,
        // then apply the overall SPEED setting
        double biggest = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb))));
        double k = SPEED / biggest;
        flwheel.setPower(lf * k);
        blwheel.setPower(lb * k);
        frwheel.setPower(rf * k);
        brwheel.setPower(rb * k);
    }

    /**
     * SLOW DOWN near the target: the speed limit shrinks from MAX_POWER (far away)
     * to MIN_DRIVE_POWER (at the target), starting SLOW_DOWN_DIST inches out.
     * Returns {forward, left} scaled so the robot never goes faster than that limit.
     */
    private double[] slowNearTarget(double forward, double left, double distLeft) {
        double limit = MIN_DRIVE_POWER + (MAX_POWER - MIN_DRIVE_POWER)
                * Range.clip(distLeft / SLOW_DOWN_DIST, 0, 1);
        double mag = Math.hypot(forward, left);
        if (mag > limit && mag > 0) {
            forward *= limit / mag;
            left    *= limit / mag;
        }
        return new double[]{forward, left};
    }

    private void stopDrive() { setMecanum(0, 0, 0, 1.0); }

    private void intakeOn()  { intakemotor.setPower(1); leftintake.setPower(1); rightintake.setPower(1); }
    private void intakeOff() { intakemotor.setPower(0); leftintake.setPower(0); rightintake.setPower(0); }

    /** Keeps an angle between -180 and +180 degrees (in radians) so the robot turns the short way. */
    private double angleWrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private boolean timeToReturn() {
        return useTimer && matchTimer.isLessThan(RETURN_TIME);
    }

    private void waitSeconds(double s) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < s) {
            localizer.update();
            idle();
        }
    }

    private void showTelemetry(Pose target) {
        Pose now = localizer.pose();
        telemetry.addData("Status", status);
        telemetry.addData("Pollen", pollenCollected + " / " + maxPollen);
        telemetry.addData("Time left", "%.1f s", matchTimer.getRemaining());
        telemetry.addData("Camera", "%d boxes, %d rejected%s, seen %d frames",
                lastRaw, lastRejected, lastRejected > 0 ? " (" + lastReject + ")" : "", seenFrames);
        if (lastBlob != null) {
            telemetry.addData("Ball box", "w %d  y %d  (zone: w>=%d or y>=%d)",
                    lastBlob.w, lastBlob.y, INTAKE_ZONE_WIDTH, INTAKE_ZONE_Y);
            if (lastBlob.field != null) {
                telemetry.addData("Ball on field", "x %.0f  y %.0f", lastBlob.field.getX(), lastBlob.field.getY());
            }
        }
        telemetry.addData("Robot", "x %.1f  y %.1f  h %.0f°",
                now.x(), now.y(), Math.toDegrees(now.heading()));
        if (target != null) {
            telemetry.addData("Target", "x %.1f  y %.1f  h %.0f°",
                    target.x(), target.y(), Math.toDegrees(target.heading()));
        }
        telemetry.update();
    }
}