package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Red Far", group="State")
public class AutoRedFarPedro extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    // Declare OpMode members.
    public final ElapsedTime runtime = new ElapsedTime();
    public final ElapsedTime stateTime = new ElapsedTime();
    private final BatBotSmart robot = new BatBotSmart();

    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    public boolean isRed = true;

    public enum FarStates {
        DETECT_PATTERN,
        MOVE_TO_FIRE_1,
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
    public Pose shootingPose = new Pose(83, 17, Math.toRadians(63));
    public Pose toBallsPose = new Pose(89, 22, Math.toRadians(0));
    public Pose pickedUpBallsPose = new Pose(115, 22, Math.toRadians(0));
    public Pose shootingPose2 = new Pose(87, 17, Math.toRadians(63));
    public Pose parkPose = new Pose(100, 0, Math.toRadians(0));

    public PathChain toFire1;
    public PathChain toLoad;
    public PathChain loadingBalls;
    public PathChain toFire2;
    public PathChain toPark;
    public FarStates currentState = FarStates.DETECT_PATTERN;
    public int[] shotOrder = STEMperFiConstants.AUTO_SHOTS_21_GPP;

    @Override
    public void init() {
        robot.init(hardwareMap, gamepad1, gamepad2, joinedTelemetry, blackboard);
        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);

        toFire1 = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, shootingPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), shootingPose.getHeading()).build();

        toLoad = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, toBallsPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), toBallsPose.getHeading()).build();

        loadingBalls = follower.pathBuilder()
                .addPath(new BezierLine(toBallsPose, pickedUpBallsPose))
                .setLinearHeadingInterpolation(toBallsPose.getHeading(), pickedUpBallsPose.getHeading()).build();

        toFire2 = follower.pathBuilder()
                .addPath(new BezierLine(pickedUpBallsPose, shootingPose))
                .setLinearHeadingInterpolation(pickedUpBallsPose.getHeading(), shootingPose.getHeading()).build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, parkPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), parkPose.getHeading()).build();

        if (robot.detectAutoPattern()) {
            joinedTelemetry.addData("pattern", robot.pattern);
        } else {
            joinedTelemetry.addData("pattern", "no pattern");
        }
        joinedTelemetry.addData(">", "Robot Heading = %4.0f", robot.odo.getHeading(AngleUnit.DEGREES));
        joinedTelemetry.update();
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
        robot.clearIndexer();

        joinedTelemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        if (robot.detectAutoPattern()) {
            joinedTelemetry.addData("pattern", robot.pattern);
        } else {
            joinedTelemetry.addData("pattern", "no pattern");
        }
        joinedTelemetry.addData(">", "Robot Heading = %4.0f", robot.odo.getHeading(AngleUnit.DEGREES));

        int currentIndex = robot.indexerIndex;
//        robot.indexerContents[0] = STEMperFiConstants.GB_LED_PINKISH;
//        robot.indexerContents[1] = STEMperFiConstants.GB_LED_BLUE;
//        robot.indexerContents[2] = STEMperFiConstants.GB_LED_YELLOW;
        if (currentIndex == 0) {
            robot.indexerContents[0] = STEMperFiConstants.GB_LED_GREEN;
        } else {
            robot.indexerContents[currentIndex] = STEMperFiConstants.GB_LED_PURPLE;
        }
        robot.startLoop();
        robot.indexer(true);

        joinedTelemetry.update();
    }
    @Override

    public void start() {
        currentState = FarStates.MOVE_TO_FIRE_1;
        robot.shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_MED;
        robot.hoodPosition = STEMperFiConstants.HOOD_RELATIVE_ANGLE_MED;
        if (STEMperFiConstants.PATTERN_22_PGP.equals(robot.pattern)) {
            shotOrder = STEMperFiConstants.AUTO_SHOTS_22_PGP;
        } else if (STEMperFiConstants.PATTERN_23_PPG.equals(robot.pattern)) {
            shotOrder = STEMperFiConstants.AUTO_SHOTS_23_PPG;
        }
        robot.setIndexerPosition(shotOrder[0]);

        robot.limelight.pipelineSwitch(robot.isRed ? STEMperFiConstants.LIMELIGHT_PIPELINE_RED : STEMperFiConstants.LIMELIGHT_PIPELINE_BLUE);
        robot.limelight.start();

        follower.followPath(toFire1, 0.3, true);
        runtime.reset();
        stateTime.reset();
    }

    @Override
    public void loop() {
        robot.startLoop();

        // INTAKE
        robot.intake();

        // FLYWHEEL
        robot.flywheel();

        if (currentState != FarStates.PARK && currentState != FarStates.MOVE_TO_BALLS && currentState != FarStates.COLLECT_BALLS) {
            robot.detectGoal(0);
        } else {
            robot.turretTargetPosition = 0;
        }

        robot.setTurretPower();

        follower.update(); // Update Pedro Pathing

        currentState = autonomousPathUpdate() ;

        // Log values to Panels and Driver Station
        joinedTelemetry.addData("State", currentState);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        joinedTelemetry.update();
        if (currentState == FarStates.END) {
            stop();
        }
    }

    boolean ballShot = false;
    public FarStates fire(FarStates cur, FarStates next, int nextIndexPosition) {
        joinedTelemetry.addData("fire stateTime", stateTime.milliseconds());
        joinedTelemetry.addData("fire ballShot", ballShot);
        if (stateTime.milliseconds() > 1_500) {
            stateTime.reset();
            ballShot = false;
            return next;
        } else if (stateTime.milliseconds() > STEMperFiConstants.INTAKE_DURING_INDEXER_MOVE_DELAY_MS + (STEMperFiConstants.SHOOT_DELAY_FLIPPER_MS * 2)) {
            joinedTelemetry.addData("fire indexer", nextIndexPosition);
            robot.setIndexerPosition(nextIndexPosition);
        } else if (stateTime.milliseconds() > STEMperFiConstants.SHOOT_DELAY_FLIPPER_MS) {
            joinedTelemetry.addData("fire flipper down", STEMperFiConstants.FLIPPER_INTAKE);
            robot.flipperServo.setPosition(STEMperFiConstants.FLIPPER_INTAKE);
        } else if (robot.isLocked()) {
            joinedTelemetry.addData("fire flipper up", STEMperFiConstants.FLIPPER_SHOOT);
            robot.flipperServo.setPosition(STEMperFiConstants.FLIPPER_SHOOT);
            ballShot = true;
        } else if (!ballShot) {
            stateTime.reset();
        }
        return cur;
    }

    public FarStates autonomousPathUpdate() {
        switch (currentState) {
            case MOVE_TO_FIRE_1:
                robot.isAutoShooterTriggerPressed = false;
                robot.setIndexerPosition(shotOrder[0]);
                robot.indexerServo.setPosition(robot.indexerServoPosition);
                joinedTelemetry.addData("isBusy", follower.isBusy());
                joinedTelemetry.addData("stateTime", stateTime.milliseconds());
                joinedTelemetry.addData("onTarget", robot.isTurretStopped);
                joinedTelemetry.addData("detect", robot.isGoalDetected);
                if (!follower.isBusy() && stateTime.milliseconds() > 6_000 && robot.isTurretStopped) {
                    robot.isAutoShooterTriggerPressed = true;
                    robot.shootPressed = robot.now;
                    stateTime.reset();
                    ballShot = false;
                    return FarStates.FIRE_1;
                }
                return FarStates.MOVE_TO_FIRE_1;
            case FIRE_1:
                return fire(FarStates.FIRE_1, FarStates.FIRE_2, shotOrder[1]);
            case FIRE_2:
                return fire(FarStates.FIRE_2, FarStates.FIRE_3, shotOrder[2]);
            case FIRE_3:
                FarStates temp = fire(FarStates.FIRE_3, FarStates.MOVE_TO_BALLS, shotOrder[0]);
                if (temp == FarStates.MOVE_TO_BALLS) {
                    robot.isAutoShooterTriggerPressed = false;
                    robot.shootPressed = 0;
                    follower.followPath(toLoad, .6, true);
                }
                return temp;
            case MOVE_TO_BALLS:
                robot.indexerContents[0] = STEMperFiConstants.GB_LED_OFF;
                robot.indexerContents[1] = STEMperFiConstants.GB_LED_OFF;
                robot.indexerContents[2] = STEMperFiConstants.GB_LED_OFF;
                robot.setIndexerPosition(0);
                if (follower.isBusy()) {
                    return FarStates.MOVE_TO_BALLS;
                }
                stateTime.reset();
                robot.intakeOn = true;
                follower.followPath(loadingBalls, .2, true);
                return FarStates.COLLECT_BALLS;
            case COLLECT_BALLS:
                if (follower.isBusy()) {
                    return FarStates.COLLECT_BALLS;
                }
                stateTime.reset();
                robot.intakeOn = false;
                follower.followPath(toFire2, .6, true);
                return FarStates.MOVE_TO_FIRE_2;
            case MOVE_TO_FIRE_2:
                if (follower.isBusy()) {
                    robot.indexerContents[0] = STEMperFiConstants.GB_LED_GREEN;
                    robot.indexerContents[1] = STEMperFiConstants.GB_LED_PURPLE;
                    robot.indexerContents[2] = STEMperFiConstants.GB_LED_PURPLE;
                    robot.setIndexerPosition(shotOrder[0]);
                    return FarStates.MOVE_TO_FIRE_2;
                }
                stateTime.reset();
                robot.isAutoShooterTriggerPressed = true;
                robot.shootPressed = robot.now;
                return FarStates.FIRE_4;
            case FIRE_4:
                return fire(FarStates.FIRE_4, FarStates.FIRE_5, shotOrder[1]);
            case FIRE_5:
                return fire(FarStates.FIRE_5, FarStates.FIRE_6, shotOrder[2]);
            case FIRE_6:
                FarStates temp2 = fire(FarStates.FIRE_6, FarStates.PARK, shotOrder[0]);
                if (temp2 == FarStates.PARK) {
                    stateTime.reset();
                    robot.isAutoShooterTriggerPressed = false;
                    robot.shootPressed = 0;
                    follower.followPath(toPark, .6, true);
                }
                return temp2;
            case PARK:
                robot.setIndexerPosition(0);
                robot.hoodPosition = STEMperFiConstants.HOOD_RELATIVE_ANGLE_SHORT;
                robot.hoodServo.setPosition(robot.hoodPosition);
                robot.shooterSpeed = 0;
                robot.fwBotMotor.set(0);
                robot.fwBotMotor.stopMotor();
                if (follower.isBusy()) {
                    return FarStates.PARK;
                }
                return FarStates.END;
        }
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return FarStates.END;
    }



//    public static class Paths {
//        public PathChain toFire1;
//        public PathChain toLoad;
//        public PathChain loadingBalls;
//        public PathChain toFire2;
//        public PathChain toPark;
//
//        public Paths(Follower follower) {
//            toFire1 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(83, 8.25), new Pose(83.000, 17)))
//                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(63)).build();
//
//            toLoad = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(83.000, 17), new Pose(89.000, 22)))
//                    .setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(0)).build();
//
//            loadingBalls = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(89.000, 22), new Pose(115.000, 22)))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();
//
//            toFire2 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(115, 22), new Pose(83.000, 17)))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(63)).build();
//
//            toPark = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(83.000, 17), new Pose(110, 0)))
//                    .setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(0)).build();
//        }
//    }
}
