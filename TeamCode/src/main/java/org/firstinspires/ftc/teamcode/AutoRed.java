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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Auto Red Shoot", group="Meet 2 Shoot")
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

    public void shootLoop(double nextBallIndex) {
        robot.flipperServo.setPosition(STEMperFiConstants.FLIPPER_SHOOT);
        waitFlyWheel(1_000);
        robot.flipperServo.setPosition(STEMperFiConstants.FLIPPER_INTAKE);
        waitFlyWheel(1_000);
        robot.indexerServoPosition = nextBallIndex;
        robot.indexer(true);
        waitFlyWheel(1_000);
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, gamepad1, gamepad2, telemetry);
        robot.odo.recalibrateIMU();
        robot.odo.resetPosAndIMU();

        while (opModeInInit()) {
            robot.odo.resetPosAndIMU();
            sleep(250);
            telemetry.addData(">", "Robot Heading = %4.0f", robot.odo.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            robot.startLoop();
            robot.indexer(true);
        }
        robot.indexerServoPosition = STEMperFiConstants.INDEX_1;
        robot.indexer(true);
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
        time = 600;
        driveTime(speed, time);
        robot.intakeMotor.setPower(1);
        robot.shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_SHORT;
        waitFlyWheel(4_000);
    //Ball 1
        shootLoop(STEMperFiConstants.INDEX_2);
    // Ball 2
        shootLoop(STEMperFiConstants.INDEX_3);
    // Ball 3
        shootLoop(STEMperFiConstants.INDEX_1);
        robot.shooterSpeed = 0;
        robot.flywheel();
        robot.intakeMotor.setPower(0);

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
