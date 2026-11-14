package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/*
  ROBOT NAMES - one place for every Driver Hub config name the pollen code uses.
  ------------------------------------------------------------------------------
  Each part has a list of names. The code uses the FIRST name it finds in the active
  config, so it works with the new "BioBuzz Robot" config AND the old "Starter Bot" one.
  If you rename something on the Driver Hub, add the new name to the FRONT of its list.

  "BioBuzz Robot" config (Control Hub), from the photos:
     motor 0 = blwheel   motor 1 = flwheel   motor 2 = transfer   motor 3 = intake2
     I2C 1   = "husky lens"  (with a space)
*/
public final class RobotNames {
    private RobotNames() {}

    // Drive wheels
    public static final String[] FL = {"flwheel", "fl"};
    public static final String[] FR = {"frwheel", "fr"};
    public static final String[] BL = {"blwheel", "bl"};
    public static final String[] BR = {"brwheel", "br"};

    // Camera
    public static final String[] HUSKY = {"husky lens", "huskylens", "husylens", "husky"};

    // Intake: EVERY one of these that exists is turned on together.
    // "transfer" runs WITH the intake so the ball is carried on into the robot.
    // To stop the transfer from running with the intake, take "transfer" out of this list.
    public static final String[] INTAKE_MOTORS = {"intake1", "intake", "intakemotor", "intake2", "transfer"};
    public static final String[] INTAKE_SERVOS = {"leftintake", "rightintake"};

    /** Direction for each intake part. If one pushes pollen OUT, flip it here. */
    public static DcMotorSimple.Direction intakeDir(String name) {
        switch (name) {
            case "intake2":     return PollenCollector.INTAKE2_DIR;
            case "transfer":    return PollenCollector.TRANSFER_DIR;
            case "leftintake":  return PollenCollector.LEFT_INTAKE_DIR;
            case "rightintake": return PollenCollector.RIGHT_INTAKE_DIR;
            default:            return PollenCollector.INTAKE_MOTOR_DIR;
        }
    }

    /** First name from the list that is in the config, or null if none are. */
    public static <T> String pick(HardwareMap hw, Class<T> cls, String[] names) {
        for (String n : names) if (hw.tryGet(cls, n) != null) return n;
        return null;
    }

    /** Get a part that MUST exist. Stops with a clear message if none of the names are in the config. */
    public static <T> T need(HardwareMap hw, Class<T> cls, String[] names, String what) {
        String n = pick(hw, cls, names);
        if (n == null) throw new IllegalArgumentException("Can't find the " + what
                + " in the Driver Hub config. Names tried: " + String.join(", ", names)
                + ". Rename it on the Driver Hub or add its name in RobotNames.java");
        return hw.get(cls, n);
    }

    /** All the intake parts found in the config, run together. */
    public static class Intake {
        public final List<DcMotor> motors = new ArrayList<>();
        public final List<CRServo> servos = new ArrayList<>();
        public final List<String> names = new ArrayList<>();

        public Intake(HardwareMap hw) {
            for (String n : INTAKE_MOTORS) {
                DcMotor m = hw.tryGet(DcMotor.class, n);
                if (m != null) { m.setDirection(intakeDir(n)); motors.add(m); names.add(n); }
            }
            for (String n : INTAKE_SERVOS) {
                CRServo s = hw.tryGet(CRServo.class, n);
                if (s != null) { s.setDirection(intakeDir(n)); servos.add(s); names.add(n); }
            }
        }

        public void setPower(double p) {
            for (DcMotor m : motors) m.setPower(p);
            for (CRServo s : servos) s.setPower(p);
        }

        public String describe() {
            return names.isEmpty() ? "NONE FOUND - intake won't spin! Check names in RobotNames.java"
                                   : String.join(", ", names);
        }
    }
}
