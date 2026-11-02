package org.firstinspires.ftc.teamcode.TestComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autos.PollenCollector;

/*
  StarterBot Direction Test
  -------------------------
  Uses the SAME names and directions as "Pollen Collector (P)", so whatever you
  fix here (in PollenCollector.java's direction settings) fixes the auto too.

  PUT THE ROBOT ON BLOCKS (wheels off the ground) for steps 1 and 3.

  1. ONE WHEEL AT A TIME (hold the button):
       X = fl (front left)    Y = fr (front right)
       A = bl (back left)     B = br (back right)
     -> The wheel you expect should spin, and spin FORWARD (top of the wheel rolling toward the front/intake).
        Wrong wheel spins  -> the cable/config name is mixed up.
        Right wheel, backwards -> flip that wheel's *_DIR in PollenCollector.java.

  2. WHOLE ROBOT (on the floor):
       D-pad UP    = drive forward      D-pad DOWN  = drive backward
       D-pad LEFT  = turn left          D-pad RIGHT = turn right
     -> If "turn" makes it slide sideways, or "forward" slides sideways, step 1 has a problem.

  3. INTAKE (hold):
       Right bumper  = intakemotor only
       Left bumper   = leftintake servo only
       Right trigger = rightintake servo only
       Left trigger  = EVERYTHING together (what the auto does)
     -> Each part should pull pollen IN. If one pushes OUT, flip its *_DIR.
*/
@TeleOp(name = "StarterBot Direction Test", group = "Testing")
public class StarterBotDirectionTest extends LinearOpMode {

    private static final double WHEEL_POWER = 0.3;

    @Override
    public void runOpMode() {
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        CRServo leftintake  = hardwareMap.get(CRServo.class, "leftintake");
        CRServo rightintake = hardwareMap.get(CRServo.class, "rightintake");

        fl.setDirection(PollenCollector.FL_DIR);
        fr.setDirection(PollenCollector.FR_DIR);
        bl.setDirection(PollenCollector.BL_DIR);
        br.setDirection(PollenCollector.BR_DIR);
        intakemotor.setDirection(PollenCollector.INTAKE_MOTOR_DIR);
        leftintake.setDirection(PollenCollector.LEFT_INTAKE_DIR);
        rightintake.setDirection(PollenCollector.RIGHT_INTAKE_DIR);

        telemetry.addLine("Robot on BLOCKS. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double flP = 0, frP = 0, blP = 0, brP = 0;
            String doing = "nothing (hold a button)";

            // 1. one wheel at a time
            if (gamepad1.x) { flP = WHEEL_POWER; doing = "fl only - should be FRONT LEFT, spinning forward"; }
            if (gamepad1.y) { frP = WHEEL_POWER; doing = "fr only - should be FRONT RIGHT, spinning forward"; }
            if (gamepad1.a) { blP = WHEEL_POWER; doing = "bl only - should be BACK LEFT, spinning forward"; }
            if (gamepad1.b) { brP = WHEEL_POWER; doing = "br only - should be BACK RIGHT, spinning forward"; }

            // 2. whole robot (same wheel math as the auto)
            if (gamepad1.dpad_up)    { flP = frP = blP = brP =  WHEEL_POWER; doing = "FORWARD - all wheels forward"; }
            if (gamepad1.dpad_down)  { flP = frP = blP = brP = -WHEEL_POWER; doing = "BACKWARD"; }
            if (gamepad1.dpad_left)  { flP = blP = -WHEEL_POWER; frP = brP =  WHEEL_POWER; doing = "TURN LEFT - should spin in place"; }
            if (gamepad1.dpad_right) { flP = blP =  WHEEL_POWER; frP = brP = -WHEEL_POWER; doing = "TURN RIGHT - should spin in place"; }

            fl.setPower(flP); fr.setPower(frP); bl.setPower(blP); br.setPower(brP);

            // 3. intake
            boolean all = gamepad1.left_trigger > 0.5;
            intakemotor.setPower((gamepad1.right_bumper || all) ? 1 : 0);
            leftintake.setPower((gamepad1.left_bumper || all) ? 1 : 0);
            rightintake.setPower((gamepad1.right_trigger > 0.5 || all) ? 1 : 0);
            if (gamepad1.right_bumper) doing = "intakemotor only - should pull IN";
            if (gamepad1.left_bumper)  doing = "leftintake servo only - should pull IN";
            if (gamepad1.right_trigger > 0.5) doing = "rightintake servo only - should pull IN";
            if (all) doing = "WHOLE INTAKE - should pull pollen IN";

            telemetry.addData("Running", doing);
            telemetry.addLine();
            telemetry.addLine("Wheels:  X=fl  Y=fr  A=bl  B=br");
            telemetry.addLine("Drive:   Dpad up/down = fwd/back, left/right = turn");
            telemetry.addLine("Intake:  RB=motor  LB=left servo  RT=right servo  LT=all");
            telemetry.addLine("Wrong direction? Flip it in PollenCollector.java (*_DIR)");
            telemetry.update();
        }
    }
}
