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

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(5.3965837373508245);
        c.yPodOffset.set(-2.0909690856933594);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.14521381753189574);
                Controller secondaryTranslationalForward = Controller.proportional(0.05365262539336864);
                Controller primaryTranslationalLateral = Controller.proportional(0.24201770584073373);
                Controller secondaryTranslationalLateral = Controller.proportional(0.0894190754759497);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01741208391886584));
                c.brake.set(Controller.proportionalFeedforward(0.014800271331035963));

                c.headingFeedback.set(Controller.proportional(2.869534816726525));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.042231097642884845, 0.00474471021361543));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05666071299572067, 0.05978620592825494));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012967842800920629, 0.001074514342163378));

                c.maxAchievableForwardVelocity.set(58.349461146294495);
                c.maxAchievableStrafeVelocity.set(51.0566914277514);
                c.naturalForwardDeceleration.set(39.008161610293264);
                c.naturalStrafeDeceleration.set(61.48120676014804);
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