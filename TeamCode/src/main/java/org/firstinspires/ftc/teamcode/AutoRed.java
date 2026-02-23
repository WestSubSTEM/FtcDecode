package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Auto Red Short", group="Meet 3 Short")
public class AutoRed extends LinearOpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    private final BatBot robot = new BatBot();

    public boolean isRed = true;

    public void waitFlyWheel(long timeToWait) {
        long waitStart = System.currentTimeMillis();
        do {
            robot.startLoop();
            robot.flywheel();
        } while (opModeIsActive() && System.currentTimeMillis() - waitStart < timeToWait);
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, gamepad1, gamepad2, telemetry);
        robot.odo.recalibrateIMU();
        robot.odo.resetPosAndIMU();

        robot.init(hardwareMap, gamepad1, gamepad2, telemetry);
        robot.odo.recalibrateIMU();
        robot.odo.resetPosAndIMU();
        robot.limelight.pipelineSwitch(STEMperFiConstants.LIMELIGHT_PIPELINE_AUTO);
        robot.isRed = isRed;
        if (isRed) {
            blackboard.put(STEMperFiConstants.BLACKBOARD_KEY_ALLIANCE, STEMperFiConstants.ALLIANCE_RED);
            robot.setAllLedsSolid(Color.RED);
        } else {
            blackboard.put(STEMperFiConstants.BLACKBOARD_KEY_ALLIANCE, STEMperFiConstants.ALLIANCE_BLUE);
            robot.setAllLedsSolid(Color.BLUE);
        }

        while (opModeInInit()) {
           // robot.turretPosition = STEMperFiConstants.TURRET_CENTER;
          //  robot.turretServo.setPosition(robot.turretPosition);
            if (!robot.indexMoved) {
                robot.odo.resetPosAndIMU();
                telemetry.addData("resetPosAndIMU: ", robot.indexMoved);
            }
            sleep(250);
            robot.odo.update();
            if (robot.detectAutoPattern()) {
                telemetry.addData("pattern", robot.pattern);
            } else {
                telemetry.addData("pattern", "no pattern");
            }
            telemetry.addData(">", "Robot Heading = %4.0f", robot.odo.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            robot.startLoop();
            robot.indexer(true);
        }
        runtime.reset();
        robot.limelight.start();
        robot.setIndexerPosition(0);
        robot.startLoop();
        if (isRed) {
          //  robot.turretPosition = STEMperFiConstants.TURRET_CENTER -.2;
        } else {
           // robot.turretPosition = STEMperFiConstants.TURRET_CENTER +.3;
        }
       // robot.turretServo.setPosition(robot.turretPosition);
//        robot.shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_SHORT;
//        waitFlyWheel(4_000);

        robot.setIndexerPosition(0);
        int time = 1_250;
        double speed = 0.4;
        if (isRed) {
            strafeRightTime(speed, time);
        } else {
            strafeLeftTime(speed, time);
        }
        speed = .3;
        speed = isRed ? speed : -speed;
        time = 480;
        turnTime(speed, time);
        speed = -.3;
        time = 1200;
        driveTime(speed, time);

        robot.shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_SHORT;
        waitFlyWheel(3_000);
        while(!robot.detectAutoPattern() && runtime.milliseconds() < 1_000 ) {
            robot.startLoop();
            robot.flywheel();
            sleep(20);
        }
        blackboard.put(STEMperFiConstants.BLACKBOARD_KEY_PATTERN, robot.pattern);
        telemetry.addData("pattern", robot.pattern);
        telemetry.update();
        if (isRed) {
            robot.limelight.pipelineSwitch(STEMperFiConstants.LIMELIGHT_PIPELINE_RED);
        } else {
            robot.limelight.pipelineSwitch(STEMperFiConstants.LIMELIGHT_PIPELINE_BLUE);
        }
        robot.limelight.start();
       // robot.turretPosition = STEMperFiConstants.TURRET_CENTER;
//robot.turretServo.setPosition(robot.turretPosition);

        robot.intakeMotor.setPower(1);
        robot.shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_SHORT;
        waitFlyWheel(4_000);
        runtime.reset();
        robot.startLoop();
        while(!robot.detectGoal(0) && runtime.milliseconds() < 2_000 ) {
            robot.startLoop();
            robot.flywheel();
            telemetry.addData("Target", "off");
            sleep(20);
            telemetry.update();
        }
        telemetry.addData("Target", "on");
        telemetry.update();
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
    //    robot.turretServo.setPosition(robot.turretPosition);

        speed = .2;
        time = 250;
        driveTime(speed, time);

        speed = 0.4;
        time = 1_000;
        if (!isRed) {
            strafeRightTime(speed, time);
        } else {
            strafeLeftTime(speed, time);
        }
        speed = .2;
        time = 750;
        driveTime(speed, time);


        sleep(5);
    }


    // **********  HIGH Level driving functions.  ********************
    public void turnTime(double maxTurnSpeed, long timeMs) {
        long start = System.currentTimeMillis();
        robot.mecanum.driveRobotCentric(0, 0, maxTurnSpeed);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
            sleep(100);
            robot.odo.update();
            telemetry.addData("current y: ", robot.odo.getPosY(DistanceUnit.MM));
            telemetry.update();
        }
        robot.mecanum.driveRobotCentric(0, 0, 0);
    }

    public void driveTime(double maxDriveSpeed, long timeMs) {
        long start = System.currentTimeMillis();
        robot.mecanum.driveRobotCentric(0, maxDriveSpeed, 0);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
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
            telemetry.addData("current y: ", robot.odo.getPosY(DistanceUnit.MM));
            telemetry.update();
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
            telemetry.addData("current y: ", robot.odo.getPosY(DistanceUnit.MM));
            telemetry.update();
        }

        robot.mecanum.driveRobotCentric(0, 0, 0);
    }

}
