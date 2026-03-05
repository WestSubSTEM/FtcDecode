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

@Autonomous(name="Auto Red Far Pedro", group="State")
public class AutoRedFarPedro extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private AutoRedFarPedro.Paths paths; // Paths defined in the Paths class
    // Declare OpMode members.
    public final ElapsedTime runtime = new ElapsedTime();
    public final ElapsedTime stateTime = new ElapsedTime();
    private final BatBotSmart robot = new BatBotSmart();

    private final com.qualcomm.robotcore.hardware.Gamepad autoGamepad = new com.qualcomm.robotcore.hardware.Gamepad();

    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

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
        MOVE_TO_CORNER,
        COLLECT_CORNER,
        COLLECT_CORNER_TWIST,
        MOVE_TO_FIRE_3,
        FIRE_7,
        FIRE_8,
        FIRE_9,
        PARK,
        END
    }



    public Pose startingPose = new Pose(83, 8.25, Math.toRadians(90));

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

        paths = new AutoRedFarPedro.Paths(follower); // Build paths

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

        robot.indexerContents[0] = STEMperFiConstants.GB_LED_GREEN;
        robot.indexerContents[1] = STEMperFiConstants.GB_LED_PURPLE;
        robot.indexerContents[2] = STEMperFiConstants.GB_LED_PURPLE;

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
        joinedTelemetry.update();
        robot.startLoop();
        robot.indexer(true);

        joinedTelemetry.update();
    }
    @Override

    public void start() {
        //robot.gp2 = new GamepadEx(autoGamepad);
        currentState = FarStates.MOVE_TO_FIRE;
        robot.shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_MED;
        robot.hoodPosition = STEMperFiConstants.HOOD_RELATIVE_ANGLE_MED;
        if (STEMperFiConstants.PATTERN_22_PGP.equals(robot.pattern)) {
            shotOrder = STEMperFiConstants.AUTO_SHOTS_22_PGP;
        } else if (STEMperFiConstants.PATTERN_23_PPG.equals(robot.pattern)) {
            shotOrder = STEMperFiConstants.AUTO_SHOTS_23_PPG;
        }
        robot.limelight.pipelineSwitch(robot.isRed ? STEMperFiConstants.LIMELIGHT_PIPELINE_RED : STEMperFiConstants.LIMELIGHT_PIPELINE_BLUE);
        robot.limelight.start();
        robot.setIndexerPosition(shotOrder[0]);
        follower.followPath(paths.toFire1Red, 0.3, true);
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

        if (currentState != FarStates.PARK) {
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
        } else if (stateTime.milliseconds() > STEMperFiConstants.INTAKE_DURING_INDEXER_MOVE_DELAY_MS + (STEMperFiConstants.SHOOT_DELAY_INDEX_MS * 2)) {
            joinedTelemetry.addData("fire indexer", nextIndexPosition);
            robot.setIndexerPosition(nextIndexPosition);
        } else if (stateTime.milliseconds() > STEMperFiConstants.SHOOT_DELAY_INDEX_MS*2) {
            joinedTelemetry.addData("fire flipper down", STEMperFiConstants.FLIPPER_INTAKE);
            robot.flipperServo.setPosition(STEMperFiConstants.FLIPPER_INTAKE);
        } else if (robot.lockLed.getPosition() != STEMperFiConstants.GB_LED_OFF) {
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
            case MOVE_TO_FIRE:
                robot.autoTriggerPressed = false;
                robot.setIndexerPosition(shotOrder[0]);
                robot.indexerServo.setPosition(robot.indexerServoPosition);
                joinedTelemetry.addData("isBusy", follower.isBusy());
                joinedTelemetry.addData("stateTime", stateTime.milliseconds());
                joinedTelemetry.addData("onTarget", robot.isTurretStopped);
                joinedTelemetry.addData("detect", robot.isGoalDetected);
                if (!follower.isBusy() && stateTime.milliseconds() > 6_000 && robot.isTurretStopped) {
                    robot.autoTriggerPressed = true;
                    stateTime.reset();
                    ballShot = false;
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
                    robot.autoTriggerPressed = false;
                    follower.followPath(paths.toLoadRed, .6, true);
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
                follower.followPath(paths.loadRed, .2, true);
                return FarStates.COLLECT_BALLS;
            case COLLECT_BALLS:
                if (follower.isBusy()) {
                    return FarStates.COLLECT_BALLS;
                }
                stateTime.reset();
                robot.intakeOn = false;
                follower.followPath(paths.toFire2Red, .6, true);
                return FarStates.MOVE_TO_FIRE_2;
            case MOVE_TO_FIRE_2:
                if (follower.isBusy()) {
                    robot.setIndexerPosition(shotOrder[0]);
                    return FarStates.MOVE_TO_FIRE_2;
                }
                stateTime.reset();
                robot.autoTriggerPressed = true;
                return FarStates.FIRE_4;
            case FIRE_4:
                return fire(FarStates.FIRE_4, FarStates.FIRE_5, shotOrder[1]);
            case FIRE_5:
                return fire(FarStates.FIRE_5, FarStates.FIRE_6, shotOrder[2]);
            case FIRE_6:
                FarStates temp2 = fire(FarStates.FIRE_6, FarStates.PARK, shotOrder[0]);
                if (temp2 == FarStates.PARK) {
                    stateTime.reset();
                    robot.autoTriggerPressed = false;
                    follower.followPath(paths.toParkRed, .6, true);
                }
                return temp2;
//            case MOVE_TO_CORNER:
//                robot.indexerContents[0] = STEMperFiConstants.GB_LED_OFF;
//                robot.indexerContents[1] = STEMperFiConstants.GB_LED_OFF;
//                robot.indexerContents[2] = STEMperFiConstants.GB_LED_OFF;
//                if (follower.isBusy()) {
//                    return FarStates.MOVE_TO_CORNER;
//                }
//                stateTime.reset();
//                robot.intakeOn = true;
//                follower.followPath(paths.loadCorner, .2, true);
//                return FarStates.COLLECT_CORNER;
//            case COLLECT_CORNER:
//                if (stateTime.milliseconds() > 1_000) {
//                    follower.breakFollowing();
//                    Pose currentPose = follower.getPose();
//                    PathChain tempChain = follower.pathBuilder()
//                            .addPath(new BezierLine(currentPose, new Pose(120, 0)))
//                            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30)).build();
//                    follower.followPath(tempChain);
//                    stateTime.reset();
//                    return FarStates.COLLECT_CORNER_TWIST;
//                }
//                if (follower.isBusy()) {
//                    return FarStates.COLLECT_CORNER;
//                }
//            case COLLECT_CORNER_TWIST:
//                if (stateTime.milliseconds() > 1_000) {
//                    follower.breakFollowing();
//                    Pose currentPose = follower.getPose();
//                    PathChain tempChain = follower.pathBuilder()
//                            .addPath(new BezierLine(currentPose, new Pose(83.000, 17)))
//                            .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(63)).build();
//                    follower.followPath(tempChain);
//                    stateTime.reset();
//                    robot.intakeOn = false;
//                    return FarStates.MOVE_TO_FIRE_3;
//                }
//                return FarStates.COLLECT_CORNER_TWIST;
//            case MOVE_TO_FIRE_3:
//                robot.autoTriggerPressed = true;
//                if (follower.isBusy()) {
//                    return FarStates.MOVE_TO_FIRE_3;
//                }
//                stateTime.reset();
//                return FarStates.FIRE_7;
//            case FIRE_7:
//                return fire(FarStates.FIRE_7, FarStates.FIRE_8, shotOrder[1]);
//            case FIRE_8:
//                return fire(FarStates.FIRE_8, FarStates.FIRE_9, shotOrder[2]);
//            case FIRE_9:
//                FarStates temp3 = fire(FarStates.FIRE_9, FarStates.PARK, shotOrder[0]);
//                if (temp3 == FarStates.PARK) {
//                    stateTime.reset();
//                    robot.autoTriggerPressed = false;
//                    follower.followPath(paths.toParkRed, .6, true);
//                }
//                return temp3;
            case PARK:
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



    public static class Paths {
        public PathChain toFire1Red;
        public PathChain toLoadRed;
        public PathChain loadRed;
        public PathChain toFire2Red;
        public PathChain toLoadCorner;
        public PathChain loadCorner;
        public PathChain toParkRed;

        public Paths(Follower follower) {
            toFire1Red = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(83, 8.25), new Pose(83.000, 17)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(63)).build();

            toLoadRed = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(83.000, 17), new Pose(89.000, 22)))
                    .setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(0)).build();

            loadRed = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(89.000, 22), new Pose(115.000, 22)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            toFire2Red = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(115, 22), new Pose(83.000, 17)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(63)).build();

            toLoadCorner = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(83.000, 17), new Pose(110, 0)))
                    .setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(0)).build();

            loadCorner = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(110, 0), new Pose(120, 0)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            toParkRed = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(83.000, 17), new Pose(110, 0)))
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
