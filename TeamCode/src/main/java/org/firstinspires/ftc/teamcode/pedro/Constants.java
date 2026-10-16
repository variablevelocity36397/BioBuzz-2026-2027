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
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left_motor");
        c.frontRightName.set("front_right_motor");
        c.backLeftName.set("back_left_motor");
        c.backRightName.set("back_right_motor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static OTOSConfig localizerConfig = new OTOSConfig(c -> {
        c.name.set("otos");
        c.linearScalar.set(1.021758199488491);
        c.angularScalar.set(1.108346933425791);
        c.offset.set(new Pose(8.482444943405513, 6.229670967642717, Math.PI));
        c.linearUnit.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // Single gain per axis: the tuned piecewise (0.072 -> 0.194 fwd, 0.104 -> 0.283 strafe at 2.5in)
                // jumped ~2.7x at the breakpoint and caused hold oscillation
                c.forwardTranslational.set(Controller.proportional(0.07182590062990792));
                c.strafeTranslational.set(Controller.proportional(0.10446582543295702));

                c.holdPointTranslationalScaling.set(0.3);
                c.holdPointHeadingScaling.set(0.25);

                c.coast.set(Controller.proportionalFeedforward(0.01313071020908464));
                c.brake.set(Controller.proportionalFeedforward(0.011161103677721945));

                // Hold aims at (heading + brakeX * omega), so the effective D gain is headingFeedback * brakeX.
                // 2.2 * 0.5 = 1.1 was barely below tuned 4.38 * 0.309 = 1.35 and still chattered with OTOS lag
                c.headingFeedback.set(Controller.proportional(1.2)); // tuned: 4.383844534180385
                // Tuned (0.309, -0.084): the negative quadratic term made damping collapse at higher spin rates
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.3, 0));

                c.linearBrakeCoefficients.set(Matrix.diag(0.384482790981859, 0.3994166404720446));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-0.006113386074192439, -0.003032174819188887));

                c.maxAchievableForwardVelocity.set(79.17502488168367);
                c.maxAchievableStrafeVelocity.set(71.9187248711457);
                c.naturalForwardDeceleration.set(38.52433141680046);
                c.naturalStrafeDeceleration.set(60.032183268508085);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new OTOSLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}