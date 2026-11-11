package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "Tuning", name = "FlywheelTuner")
public class FlyWheelTuner extends OpMode {

    public DcMotorEx shooter;
    public double lowVelocity = 300;
    public double highvelocity = 1090;
    double curTargetVelocity = lowVelocity;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;
    static final double SPIN_UP_TOLERANCE = 12.5; // same as Teleop
    final ElapsedTime spinUpTimer = new ElapsedTime();
    double spinUpTime = -1; // -1 = still spinning up or not timing
    boolean timingSpinUp = false;
    double peakCurrent = 0; // spin-up is too fast to read live, so keep the highest


    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Init Done");
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == lowVelocity) {
                curTargetVelocity = highvelocity;
                spinUpTimer.reset();
                spinUpTime = -1;
                timingSpinUp = true;
                peakCurrent = 0;
            } else {
                curTargetVelocity = lowVelocity;
                timingSpinUp = false;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        shooter.setVelocity(curTargetVelocity);

        double curVelocity = shooter.getVelocity();
        double error = curTargetVelocity - curVelocity;
        double current = shooter.getCurrent(CurrentUnit.AMPS);
        if (timingSpinUp && spinUpTime < 0) peakCurrent = Math.max(peakCurrent, current);
        if (timingSpinUp && spinUpTime < 0 && Math.abs(error) < SPIN_UP_TOLERANCE) {
            spinUpTime = spinUpTimer.seconds();
        }

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Spin-up time to high", !timingSpinUp ? "press Y for high"
                : spinUpTime < 0 ? String.format("spinning up... %.2f s", spinUpTimer.seconds())
                : String.format("%.2f s", spinUpTime));
        // Peak near the motor's stall current = maxed out, more P won't make it faster
        telemetry.addData("Motor Current (A)", "%.2f", current);
        telemetry.addData("Peak Current during spin-up (A)", "%.2f", peakCurrent);
        telemetry.addLine("------------------------");
        telemetry.addData("Tuning P", "%.4f (Dpad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (Dpad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
