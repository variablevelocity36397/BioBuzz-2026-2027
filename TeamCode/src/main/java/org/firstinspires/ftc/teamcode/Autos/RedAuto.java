package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.curve;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Red Auto", group = "Autonomous")
public class RedAuto extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // key field positions
    private final Pose startPose = poseFactory.of(56, 8.5, 90);    // also where we prescore
    private final Pose gardenPose = poseFactory.of(9, 9, 180);
    private final Pose farShootPose = poseFactory.of(60, 132, 270);
    private final Pose flowerPose = poseFactory.of(49, 132, 90);

    // bezier control points, named after the path they shape
    private final Pose startToGardenControl1 = poseFactory.of(54.8366, 26.9209, 0);
    private final Pose startToGardenControl2 = poseFactory.of(24.7417, 9, 0);
    private final Pose gardenToFarShootControl1 = poseFactory.of(23.1216, 77.5227, 0);
    private final Pose gardenToFarShootControl2 = poseFactory.of(32.9209, 107.3735, 0);
    private final Pose gardenToFarShootControl3 = poseFactory.of(53.445, 118.2613, 0);
    private final Pose farShootToFlowerControl1 = poseFactory.of(80.4462, 104.3736, 0);
    private final Pose farShootToFlowerControl2 = poseFactory.of(39.3313, 91.8358, 0);
    private final Pose flowerToFarShootControl1 = poseFactory.of(39.0828, 91.802, 0);
    private final Pose flowerToFarShootControl2 = poseFactory.of(80.6069, 104.4089, 0);

    // piecewise heading targets, only the heading is used
    private final Pose startToGardenTurnStart = poseFactory.of(9, 9, 90);
    private final Pose startToGardenTurnEnd = poseFactory.of(9, 9, 180);
    private final Pose startToGardenHoldHeading = poseFactory.of(9, 9, 180);
    private final Pose farShootToFlowerTurn1Start = poseFactory.of(49, 132, 270);
    private final Pose farShootToFlowerTurn1End = poseFactory.of(49, 132, 135);
    private final Pose farShootToFlowerTurn2Start = poseFactory.of(49, 132, 135);
    private final Pose farShootToFlowerTurn2End = poseFactory.of(49, 132, 90);
    private final Pose flowerToFarShootTurn1Start = poseFactory.of(60, 132, 90);
    private final Pose flowerToFarShootTurn1End = poseFactory.of(60, 132, 225);
    private final Pose flowerToFarShootTurn2Start = poseFactory.of(60, 132, -135);
    private final Pose flowerToFarShootTurn2End = poseFactory.of(60, 132, 270);

    private Path startToGarden;
    private Path gardenToFarShoot;
    private Path farShootToFlower;
    private Path flowerToFarShoot;

    private AutoState currentState;
    private long stateStartTime;
    enum AutoState {
        DRIVE_TO_GARDEN,
        DRIVE_TO_FAR_SHOOT,
        DRIVE_TO_FLOWER,
        RETURN_TO_FAR_SHOOT,
        DONE
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        buildPaths();
        currentState = AutoState.DRIVE_TO_GARDEN;
    }

    @Override
    public void start() {
        enterState(AutoState.DRIVE_TO_GARDEN);
    }

    @Override
    public void loop() {
        follower.update();

        // every state except DONE is a drive; move on once the path finishes
        if (currentState != AutoState.DONE && !follower.isBusy()) {
            enterState(nextState(currentState));
        }

        telemetry.addData("State", currentState);
        telemetry.addData("State time", getStateTime());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        telemetry.addData("Busy?", follower.isBusy());
        if (follower.currentPath() != null) {
            telemetry.addData("Distance remaining", follower.distanceToEndpoint());
        }
        telemetry.update();
    }

    private void buildPaths() {
        startToGarden = curve(startPose, startToGardenControl1, startToGardenControl2, gardenPose)
                .heading(Interpolator.piecewise()
                        .until(0.5, Interpolator.linear(startToGardenTurnStart, startToGardenTurnEnd))
                        .until(1, Interpolator.constant(startToGardenHoldHeading)));

        gardenToFarShoot = curve(gardenPose, gardenToFarShootControl1, gardenToFarShootControl2, gardenToFarShootControl3, farShootPose)
                .linear(gardenPose, farShootPose);

        farShootToFlower = curve(farShootPose, farShootToFlowerControl1, farShootToFlowerControl2, flowerPose)
                .heading(Interpolator.piecewise()
                        .until(0.5, Interpolator.linear(farShootToFlowerTurn1Start, farShootToFlowerTurn1End))
                        .until(1, Interpolator.linear(farShootToFlowerTurn2Start, farShootToFlowerTurn2End)));

        flowerToFarShoot = curve(flowerPose, flowerToFarShootControl1, flowerToFarShootControl2, farShootPose)
                .heading(Interpolator.piecewise()
                        .until(0.5, Interpolator.linear(flowerToFarShootTurn1Start, flowerToFarShootTurn1End))
                        .until(1, Interpolator.linear(flowerToFarShootTurn2Start, flowerToFarShootTurn2End)));
    }

    private void enterState(AutoState newState) {
        currentState = newState;
        stateStartTime = System.currentTimeMillis();

        switch (newState) {
            case DRIVE_TO_GARDEN:
                follower.follow(startToGarden);
                break;
            case DRIVE_TO_FAR_SHOOT:
                follower.follow(gardenToFarShoot);
                break;
            case DRIVE_TO_FLOWER:
                follower.follow(farShootToFlower);
                break;
            case RETURN_TO_FAR_SHOOT:
                follower.follow(flowerToFarShoot);
                break;
            case DONE:
                break;
        }
    }

    // order of the auto routine
    private AutoState nextState(AutoState state) {
        switch (state) {
            case DRIVE_TO_GARDEN:
                return AutoState.DRIVE_TO_FAR_SHOOT;
            case DRIVE_TO_FAR_SHOOT:
                return AutoState.DRIVE_TO_FLOWER;
            case DRIVE_TO_FLOWER:
                return AutoState.RETURN_TO_FAR_SHOOT;
            default:
                return AutoState.DONE;
        }
    }

    private long getStateTime() {
        return System.currentTimeMillis() - stateStartTime;
    }
}
