package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
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
        c.xPodOffset.set(-4.049118522583969);
        c.yPodOffset.set(-8.186294675812007);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4224381631913541);
                Controller secondaryTranslationalForward = Controller.proportional(0.15607961354359529);
                Controller primaryTranslationalLateral = Controller.proportional(0.6907378240996555);
                Controller secondaryTranslationalLateral = Controller.proportional(0.255209169150238);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016067525795715253));
                c.brake.set(Controller.proportionalFeedforward(0.013657396926357965));

                c.headingFeedback.set(Controller.proportional(8.041000797123479));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.042060114961665104, 0.011909995038311414));

                c.linearBrakeCoefficients.set(Matrix.diag(-3.0319336856561997E-4, 0.09098907385047646));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.004546637348232076, 0.0017542388052097819));

                c.maxAchievableForwardVelocity.set(72.76716283492676);
                c.maxAchievableStrafeVelocity.set(59.90571379507217);
                c.naturalForwardDeceleration.set(37.98423937823319);
                c.naturalStrafeDeceleration.set(69.75889903201903);
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