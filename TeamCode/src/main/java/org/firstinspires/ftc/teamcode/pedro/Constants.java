package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * UNTUNED. Retune order: MecanumTuner -> paste drivetrainConfig, PinpointTuner -> paste localizerConfig (replaces the
 * placeholder below), ForesightTuner -> paste foresightConfig. Then re-enable the tuners in Tuning and restore create().
 */
public class Constants {
    // UNTUNED placeholder (device name only) so TeleOp and the turret tests still build and the tuners can be installed.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("flwheel");
        c.frontRightName.set("frwheel");
        c.backLeftName.set("blwheel");
        c.backRightName.set("brwheel");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.3775870255597935);
        c.yPodOffset.set(-7.053276722825419);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.34180296155028334);
                Controller secondaryTranslationalForward = Controller.proportional(0.1262870611494896);
                Controller primaryTranslationalLateral = Controller.proportional(0.6481413760395075);
                Controller secondaryTranslationalLateral = Controller.proportional(0.239470919790068);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.015159501783566391));
                c.brake.set(Controller.proportionalFeedforward(0.012885576516031432));

                c.headingFeedback.set(Controller.proportional(5.6842743836816));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04422963741308023, 0.007818550363151846));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0778550765039992, 0.05768472900765055));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015869305231474928, 0.0018527745042040203));

                c.maxAchievableForwardVelocity.set(71.087941935698);
                c.maxAchievableStrafeVelocity.set(58.222904283310605);
                c.naturalForwardDeceleration.set(44.01509639799577);
                c.naturalStrafeDeceleration.set(71.85143759849068);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
