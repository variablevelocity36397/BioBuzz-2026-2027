package org.firstinspires.ftc.teamcode.TestComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.Autos.PollenCollector;
import org.firstinspires.ftc.teamcode.Autos.RobotNames;

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
       Right bumper  = 1st intake part   Left bumper = 2nd   Right trigger = 3rd
       (the screen shows which names those are - from RobotNames.java)
       Left trigger  = EVERYTHING together (what the auto does)
     -> Each part should pull pollen IN. If one pushes OUT, flip its *_DIR.
*/
@TeleOp(name = "StarterBot Direction Test", group = "Testing")
public class StarterBotDirectionTest extends LinearOpMode {

    private static final double WHEEL_POWER = 0.3;

    @Override
    public void runOpMode() {
        DcMotor fl = RobotNames.need(hardwareMap, DcMotor.class, RobotNames.FL, "front left wheel");
        DcMotor fr = RobotNames.need(hardwareMap, DcMotor.class, RobotNames.FR, "front right wheel");
        DcMotor bl = RobotNames.need(hardwareMap, DcMotor.class, RobotNames.BL, "back left wheel");
        DcMotor br = RobotNames.need(hardwareMap, DcMotor.class, RobotNames.BR, "back right wheel");
        RobotNames.Intake intakeParts = new RobotNames.Intake(hardwareMap);
        // RB = 1st intake part, LB = 2nd, RT = 3rd, LT = all of them (names shown on screen)
        List<Object> parts = new ArrayList<>(intakeParts.motors);
        parts.addAll(intakeParts.servos);

        fl.setDirection(PollenCollector.FL_DIR);
        fr.setDirection(PollenCollector.FR_DIR);
        bl.setDirection(PollenCollector.BL_DIR);
        br.setDirection(PollenCollector.BR_DIR);

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
            boolean[] press = {gamepad1.right_bumper, gamepad1.left_bumper, gamepad1.right_trigger > 0.5};
            for (int i = 0; i < parts.size(); i++) {
                boolean on = all || (i < press.length && press[i]);
                Object part = parts.get(i);
                if (part instanceof DcMotor) ((DcMotor) part).setPower(on ? 1 : 0);
                else ((CRServo) part).setPower(on ? 1 : 0);
                if (i < press.length && press[i]) doing = intakeParts.names.get(i) + " only - should pull IN";
            }
            if (all) doing = "WHOLE INTAKE - should pull pollen IN";

            telemetry.addData("Running", doing);
            telemetry.addLine();
            telemetry.addLine("Wheels:  X=fl  Y=fr  A=bl  B=br");
            telemetry.addLine("Drive:   Dpad up/down = fwd/back, left/right = turn");
            telemetry.addData("Intake parts (RB, LB, RT; LT = all)", intakeParts.describe());
            telemetry.addLine("Wrong direction? Flip it in PollenCollector.java (*_DIR)");
            telemetry.update();
        }
    }
}
