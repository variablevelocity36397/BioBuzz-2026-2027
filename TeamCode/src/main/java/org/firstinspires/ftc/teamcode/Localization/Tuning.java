package org.firstinspires.ftc.teamcode.Localization;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.Localization.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.Localization.procedures.OTOSTuner;
import org.firstinspires.ftc.teamcode.Localization.procedures.Tests;

public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
    @Tuner
    public static Procedure otosTuner() {
        return new OTOSTuner();
    }
    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), (hardwareMap -> new OTOSLocalizer(hardwareMap, Constants.localizerConfig)), null);
    }
    // Tuners go here
}
