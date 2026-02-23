package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Autonomous(name="Auto Red Far Pedro", group="Qual Far")
public class AutoRedFarPedro extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private AutoRedFarPedro.Paths paths; // Paths defined in the Paths class
    // Declare OpMode members.
    public final ElapsedTime runtime = new ElapsedTime();
    public final ElapsedTime stateTime = new ElapsedTime();
    private final BatBotSmart robot = new BatBotSmart();

    private JoinedTelemetry joinedTelemetry;

    public boolean isRed = true;

    public enum FarStates {
        DETECT_PATTERN,
        MOVE_TO_FIRE,
        FIRE_1,
        FIRE_2,
        FIRE_3,
        MOVE_TO_BALLS,
        COLLECT_BALLS,
        MOVE_TO_FIRE_2,
        FIRE_4,
        FIRE_5,
        FIRE_6,
        PARK,
        END
    }



    public Pose startingPose = new Pose(83, 8.25, Math.toRadians(90));

    public FarStates currentState = FarStates.DETECT_PATTERN;
    public int[] shotOrder = STEMperFiConstants.AUTO_SHOTS_21_GPP;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        joinedTelemetry = new JoinedTelemetry(telemetry);
        robot.init(hardwareMap, gamepad1, gamepad2, joinedTelemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);

        paths = new AutoRedFarPedro.Paths(follower); // Build paths

        if (robot.detectAutoPattern()) {
            telemetry.addData("pattern", robot.pattern);
        } else {
            telemetry.addData("pattern", "no pattern");
        }
        telemetry.addData(">", "Robot Heading = %4.0f", robot.odo.getHeading(AngleUnit.DEGREES));
        telemetry.update();
        robot.startLoop();
        robot.indexer(true);

        robot.limelight.pipelineSwitch(STEMperFiConstants.LIMELIGHT_PIPELINE_AUTO);
        robot.limelight.start();
        robot.isRed = isRed;
        if (isRed) {
            blackboard.put(STEMperFiConstants.BLACKBOARD_KEY_ALLIANCE, STEMperFiConstants.ALLIANCE_RED);
            robot.setAllLedsSolid(Color.RED);
        } else {
            blackboard.put(STEMperFiConstants.BLACKBOARD_KEY_ALLIANCE, STEMperFiConstants.ALLIANCE_BLUE);
            robot.setAllLedsSolid(Color.BLUE);
        }

        robot.indexerContents.set(0, Color.GREEN);
        robot.indexerContents.set(1, Color.PURPLE);
        robot.indexerContents.set(2, Color.PURPLE);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        if (robot.detectAutoPattern()) {
            telemetry.addData("pattern", robot.pattern);
        } else {
            telemetry.addData("pattern", "no pattern");
        }
        telemetry.addData(">", "Robot Heading = %4.0f", robot.odo.getHeading(AngleUnit.DEGREES));
        telemetry.update();
        robot.startLoop();
        robot.indexer(true);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        robot.startLoop();
        currentState = autonomousPathUpdate() ;
        robot.flywheel();
        robot.detectGoal(0);
        robot.setTurretPower();
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("State", currentState);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public FarStates fire(FarStates cur, FarStates next, int nextIndexPosition) {
        //robot.detectGoal(0);
        robot.intakeMotor.setPower(1);
        if (stateTime.milliseconds() > 1_250) {
            robot.intakeMotor.setPower(0);
            stateTime.reset();
            return next;
        } else if (stateTime.milliseconds() > 1_000) {
            robot.setIndexerPosition(nextIndexPosition);
        } else if (stateTime.milliseconds() > 500) {
            robot.flipperServo.setPosition(STEMperFiConstants.FLIPPER_INTAKE);
        } else {
            robot.flipperServo.setPosition(STEMperFiConstants.FLIPPER_SHOOT);
        }
        return cur;
    }

    public FarStates intake(FarStates cur, FarStates next, int nextIndexPosition) {
        robot.intakeMotor.setPower(1);
        //if (robot.isBallIn())
        return cur;
    }

    public FarStates autonomousPathUpdate() {
        switch (currentState) {
            case MOVE_TO_FIRE:
                robot.detectGoal(0);
                telemetry.addData("isBusy", follower.isBusy());
                telemetry.addData("stateTime", stateTime.milliseconds());
                telemetry.addData("onTarget", robot.isOnTarget);
                telemetry.addData("detect", robot.lastDetect == robot.now);
                if (!follower.isBusy() && stateTime.milliseconds() > 3_000 && robot.isOnTarget) {
                    stateTime.reset();
                    return FarStates.FIRE_1;
                }
                return FarStates.MOVE_TO_FIRE;
            case FIRE_1:
                return fire(FarStates.FIRE_1, FarStates.FIRE_2, shotOrder[1]);
            case FIRE_2:
                return fire(FarStates.FIRE_2, FarStates.FIRE_3, shotOrder[2]);
            case FIRE_3:
                FarStates temp = fire(FarStates.FIRE_3, FarStates.MOVE_TO_BALLS, shotOrder[0]);
                if (temp == FarStates.MOVE_TO_BALLS) {
                    follower.followPath(paths.toLoadRed, .4, true);
                }
            case MOVE_TO_BALLS:
                if (follower.isBusy()) {
                    return FarStates.MOVE_TO_BALLS;
                }
                stateTime.reset();
                robot.intakeMotor.setPower(1);
                follower.followPath(paths.loadRed, .4, true);
                return FarStates.COLLECT_BALLS;
            case COLLECT_BALLS:

            case PARK:
                robot.shooterSpeed = 0;
                if (follower.isBusy()) {
                    return FarStates.PARK;
                }
                stop();
                return FarStates.END;
        }
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return FarStates.END;
    }

    @Override
    public void start() {
        currentState =FarStates.MOVE_TO_FIRE;
        robot.shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_MED;
        robot.hoodPosition = STEMperFiConstants.HOOD_RELATIVE_ANGLE_MED;
        robot.flywheel();
        runtime.reset();
        stateTime.reset();
        if (STEMperFiConstants.PATTERN_22_PGP.equals(robot.pattern)) {
            shotOrder = STEMperFiConstants.AUTO_SHOTS_22_PGP;
        } else if (STEMperFiConstants.PATTERN_23_PPG.equals(robot.pattern)) {
            shotOrder = STEMperFiConstants.AUTO_SHOTS_23_PPG;
        }
        robot.limelight.pipelineSwitch(robot.isRed ? STEMperFiConstants.LIMELIGHT_PIPELINE_RED : STEMperFiConstants.LIMELIGHT_PIPELINE_BLUE);
        robot.limelight.start();
        runtime.reset();
        robot.setIndexerPosition(shotOrder[0]);
        follower.followPath(paths.toFire1Red, 0.3, true);
    }


    public static class Paths {
        public PathChain toFire1Red;
        public PathChain toLoadRed;
        public PathChain loadRed;
        public PathChain toFire2Red;
        public PathChain toParkRed;

        public Paths(Follower follower) {
            toFire1Red = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(83, 8.25), new Pose(83.000, 17)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(63)).build();

            toLoadRed = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(83.000, 17), new Pose(97.000, 35.500)))
                    .setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(0)).build();

            loadRed = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(97.000, 35.500), new Pose(129.000, 35.500)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            toFire2Red = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(129.000, 35.500), new Pose(83.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(63)).build();

            toParkRed = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(83.000, 12.000), new Pose(102.000, 28.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(0)).build();
        }
    }



//    public void waitFlyWheel(long timeToWait) {
//        long waitStart = System.currentTimeMillis();
//        do {
//            robot.startLoop();
//            robot.flywheel();
//        } while (opModeIsActive() && System.currentTimeMillis() - waitStart < timeToWait);
//    }


//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap, gamepad1, gamepad2, telemetry);
//        robot.odo.recalibrateIMU();
//        robot.odo.resetPosAndIMU();
//        robot.limelight.pipelineSwitch(STEMperFiConstants.LIMELIGHT_PIPELINE_AUTO);
//        robot.limelight.start();
//        robot.isRed = isRed;
//        if (isRed) {
//            blackboard.put(STEMperFiConstants.BLACKBOARD_KEY_ALLIANCE, STEMperFiConstants.ALLIANCE_RED);
//            robot.setAllLedsSolid(Color.RED);
//        } else {
//            blackboard.put(STEMperFiConstants.BLACKBOARD_KEY_ALLIANCE, STEMperFiConstants.ALLIANCE_BLUE);
//            robot.setAllLedsSolid(Color.BLUE);
//        }
//
//        while (opModeInInit()) {
//            if (!robot.indexMoved) {
//                robot.odo.resetPosAndIMU();
//                telemetry.addData("resetPosAndIMU: ", robot.indexMoved);
//            }
//            sleep(250);
//            robot.odo.update();
//            if (robot.detectAutoPattern()) {
//                telemetry.addData("pattern", robot.pattern);
//            } else {
//                telemetry.addData("pattern", "no pattern");
//            }
//            telemetry.addData(">", "Robot Heading = %4.0f", robot.odo.getHeading(AngleUnit.DEGREES));
//            telemetry.update();
//            robot.startLoop();
//            robot.indexer(false);
//        }
//        runtime.reset();
//        robot.setIndexerPosition(0);
//        robot.startLoop();
//        robot.shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_MED_AUTO;
//        waitFlyWheel(4_000);
//        while(!robot.detectAutoPattern() && runtime.milliseconds() < 1_000 ) {
//            robot.startLoop();
//            robot.flywheel();
//            sleep(20);
//        }
//        blackboard.put(STEMperFiConstants.BLACKBOARD_KEY_PATTERN, robot.pattern);
//        telemetry.addData("pattern", robot.pattern);
//        telemetry.update();
//       // robot.turretPosition += isRed ? .05 : -.05;
//       // robot.turretServo.setPosition(robot.turretPosition);
//        robot.limelight.pipelineSwitch(robot.isRed ? STEMperFiConstants.LIMELIGHT_PIPELINE_RED : STEMperFiConstants.LIMELIGHT_PIPELINE_BLUE);
//        robot.limelight.start();
//        runtime.reset();
//        robot.startLoop();
//        while(!robot.detectGoal(0) && runtime.milliseconds() < 2_000 ) {
//            robot.startLoop();
//            robot.flywheel();
//            telemetry.addData("Target", "off");
//            sleep(20);
//            telemetry.update();
//        }
//        telemetry.addData("Target", "on");
//        telemetry.update();
//        int[] shotOrder = STEMperFiConstants.AUTO_SHOTS_21_GPP;
//        if (STEMperFiConstants.PATTERN_22_PGP.equals(robot.pattern)) {
//            shotOrder = STEMperFiConstants.AUTO_SHOTS_22_PGP;
//        } else if (STEMperFiConstants.PATTERN_23_PPG.equals(robot.pattern)) {
//            shotOrder = STEMperFiConstants.AUTO_SHOTS_23_PPG;
//        }
//
//        waitFlyWheel(1_000);
//        robot.intakeMotor.setPower(1);
//        for (int i = 0; i < shotOrder.length; i++) {
//            robot.setIndexerPosition(shotOrder[i]);
//            waitFlyWheel(750);
//            robot.flipperServo.setPosition(STEMperFiConstants.FLIPPER_SHOOT);
//            waitFlyWheel(750);
//            robot.flipperServo.setPosition(STEMperFiConstants.FLIPPER_INTAKE);
//            waitFlyWheel(750);
//        }
//        robot.shooterSpeed = 0;
//        robot.flywheel();
//        robot.intakeMotor.setPower(0);
//        robot.setIndexerPosition(0);
//       // robot.turretPosition = STEMperFiConstants.TURRET_CENTER;
//      //  robot.turretServo.setPosition(robot.turretPosition);
//        double speed = .5;
//        long time = 600;
//        driveTime(speed, time);
//        robot.prism.clearAllAnimations();
//        sleep(100);
//        robot.prism.clearAllAnimations();
//        sleep(50);
//        robot.prism.clearAllAnimations();
//        sleep(2_00);
//    }
}
