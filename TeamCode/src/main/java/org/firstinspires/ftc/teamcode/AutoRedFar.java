package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;

@Autonomous(name="Auto Red Far", group="Meet 3 Far")
public class AutoRedFar extends LinearOpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    private final BatBot robot = new BatBot();

    public boolean isRed = true;

    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    public void waitFlyWheel(long timeToWait) {
        long waitStart = System.currentTimeMillis();
        do {
            robot.startLoop();
            robot.flywheel();
        } while (opModeIsActive() && System.currentTimeMillis() - waitStart < timeToWait);
    }


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, gamepad1, gamepad2, joinedTelemetry, blackboard);
        robot.odo.recalibrateIMU();
        robot.odo.resetPosAndIMU();
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

        while (opModeInInit()) {
            if (!robot.indexMoved) {
                robot.odo.resetPosAndIMU();
                joinedTelemetry.addData("resetPosAndIMU: ", robot.indexMoved);
            }
            sleep(250);
            robot.odo.update();
            if (robot.detectAutoPattern()) {
                joinedTelemetry.addData("pattern", robot.pattern);
            } else {
                joinedTelemetry.addData("pattern", "no pattern");
            }
            joinedTelemetry.addData(">", "Robot Heading = %4.0f", robot.odo.getHeading(AngleUnit.DEGREES));
            joinedTelemetry.update();
            robot.startLoop();
            robot.indexer(false);
        }
        runtime.reset();
        robot.setIndexerPosition(0);
        robot.startLoop();
        robot.shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_MED_AUTO;
        waitFlyWheel(4_000);
        while(!robot.detectAutoPattern() && runtime.milliseconds() < 1_000 ) {
            robot.startLoop();
            robot.flywheel();
            sleep(20);
        }
        blackboard.put(STEMperFiConstants.BLACKBOARD_KEY_PATTERN, robot.pattern);
        joinedTelemetry.addData("pattern", robot.pattern);
        joinedTelemetry.update();
       // robot.turretPosition += isRed ? .05 : -.05;
       // robot.turretServo.setPosition(robot.turretPosition);
        robot.limelight.pipelineSwitch(robot.isRed ? STEMperFiConstants.LIMELIGHT_PIPELINE_RED : STEMperFiConstants.LIMELIGHT_PIPELINE_BLUE);
        robot.limelight.start();
        runtime.reset();
        robot.startLoop();
        while(!robot.detectGoal(0) && runtime.milliseconds() < 2_000 ) {
            robot.startLoop();
            robot.flywheel();
            joinedTelemetry.addData("Target", "off");
            sleep(20);
            joinedTelemetry.update();
        }
        joinedTelemetry.addData("Target", "on");
        joinedTelemetry.update();
        int[] shotOrder = STEMperFiConstants.AUTO_SHOTS_21_GPP;
        if (STEMperFiConstants.PATTERN_22_PGP.equals(robot.pattern)) {
            shotOrder = STEMperFiConstants.AUTO_SHOTS_22_PGP;
        } else if (STEMperFiConstants.PATTERN_23_PPG.equals(robot.pattern)) {
            shotOrder = STEMperFiConstants.AUTO_SHOTS_23_PPG;
        }

        waitFlyWheel(1_000);
        robot.intakeMotor.setPower(1);
        for (int i = 0; i < shotOrder.length; i++) {
            robot.setIndexerPosition(shotOrder[i]);
            waitFlyWheel(750);
            robot.flipperServo.setPosition(STEMperFiConstants.FLIPPER_SHOOT);
            waitFlyWheel(750);
            robot.flipperServo.setPosition(STEMperFiConstants.FLIPPER_INTAKE);
            waitFlyWheel(750);
        }
        robot.shooterSpeed = 0;
        robot.flywheel();
        robot.intakeMotor.setPower(0);
        robot.setIndexerPosition(0);
       // robot.turretPosition = STEMperFiConstants.TURRET_CENTER;
      //  robot.turretServo.setPosition(robot.turretPosition);
        double speed = .5;
        long time = 600;
        driveTime(speed, time);
        robot.prism.clearAllAnimations();
        sleep(100);
        robot.prism.clearAllAnimations();
        sleep(50);
        robot.prism.clearAllAnimations();
        sleep(2_00);
    }


    // **********  HIGH Level driving functions.  ********************
    public void turnTime(double maxTurnSpeed, long timeMs) {
        long start = System.currentTimeMillis();
        robot.mecanum.driveRobotCentric(0, 0, maxTurnSpeed);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
            sleep(100);
            robot.odo.update();
            joinedTelemetry.addData("current y: ", robot.odo.getPosY(DistanceUnit.MM));
            joinedTelemetry.update();
        }
        robot.mecanum.driveRobotCentric(0, 0, 0);
    }

    public void driveTime(double maxDriveSpeed, long timeMs) {
        long start = System.currentTimeMillis();
        robot.mecanum.driveRobotCentric(0, maxDriveSpeed, 0);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
            robot.startLoop();
            sleep(100);
        }
        robot.mecanum.driveRobotCentric(0, 0, 0);
    }


    public void strafeLeftTime(double maxDriveSpeed, long timeMs) {
        long start = System.currentTimeMillis();
        robot.mecanum.driveRobotCentric(-maxDriveSpeed, 0, 0);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
            sleep(100);
            robot.odo.update();
            joinedTelemetry.addData("current y: ", robot.odo.getPosY(DistanceUnit.MM));
            joinedTelemetry.update();
        }
        robot.mecanum.driveRobotCentric(0, 0, 0);
    }

    public void strafeRightTime(double maxDriveSpeed, long timeMs) {
        long start = System.currentTimeMillis();
        robot.mecanum.driveRobotCentric(maxDriveSpeed, 0, 0);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
            sleep(100);
            robot.odo.update();
            joinedTelemetry.addData("current y: ", robot.odo.getPosY(DistanceUnit.MM));
            joinedTelemetry.update();
        }

        robot.mecanum.driveRobotCentric(0, 0, 0);
    }

}
