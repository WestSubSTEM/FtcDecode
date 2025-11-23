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

@Autonomous(name="Auto Red", group="Meet 1")
public class AutoRed extends LinearOpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
    DcMotorEx intakeMotor;
    DcMotorEx fwTopMotor, fwBotMotor;
    // input motors exactly as shown below
    MecanumDrive mecanum;
    GamepadEx pg1, gp2;
    ButtonReader square2ButtonReader, triangle2ButtonReader, circle2ButtonReader, x2ButtonReader, rightBumper2Reader, dUp2ButtonReader, dDown2ButtonReader, dLeft2ButtonReader, dRight2ButtonReader, leftStick2ButtonReader, rightStick2ButtonReader;
    MotorGroup fwMotorGroup;
    Servo flipperServo, indexerServo;
    double flipperServoPosition = STEMperFiConstants.FLIPPER_INTAKE;
    double indexerServoPosition = STEMperFiConstants.INDEX_2;
    boolean intakeOn = false;
    long serverIndexPressTimeMS = 0;

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    double oldTime = 0;
    int bucketVerticalPosition = 0;

    long odoResetTime = 0;

    public boolean isRed = true;


    @Override
    public void runOpMode() {
        robotInit();
        while (opModeInInit()) {
            odo.resetPosAndIMU();
            sleep(250);
            telemetry.addData(">", "Robot Heading = %4.0f", odo.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
//        while (opModeIsActive()) {
//            odo.update();
//            double lx = gpA.getLeftX();
//            double ly = gpA.getLeftY();
//            double rx = gpA.getRightX();
//            if (gamepad1.left_bumper || vMotor.getCurrentPosition() > 400) {
//                lx = lx / 2;
//                ly = ly / 2;
//                rx = rx / 2;
//            }
//            double degrees = Math.toDegrees(odo.getHeading());
//            mecanum.driveRobotCentric(lx, ly, rx);
//            telemetry.addData("HR:", odo.getHeading());
//            telemetry.addData("Heading: ", degrees);
//            telemetry.addData("pos", odo.getPosition());
//            telemetry.addData("pos x", odo.getPosX());
//            telemetry.addData("pos y", odo.getPosY());
//            telemetry.update();
//        }


        int time = 2_200;
        double speed = 0.4;
        if (isRed) {
            strafeRightTime(speed, time);
        } else {
            strafeLeftTime(speed, time);
        }
        speed = -0.4;
        driveStraightTime(speed, 700);
        speed = .3;
        speed = isRed ? speed : -speed;
        time = 430;
        turnTime(speed, time);
        intakeMotor.setPower(1);
        fwBotMotor.setPower(.66);
        fwTopMotor.setPower(.66);
        sleep(4_000);

        flipperServo.setPosition(STEMperFiConstants.FLIPPER_SHOOT);
        sleep(1_000);
        flipperServo.setPosition(STEMperFiConstants.FLIPPER_INTAKE);
        fwBotMotor.setPower(0);
        fwTopMotor.setPower(0);
        intakeMotor.setPower(0);
        speed = 0.4;
        time = 1_000;
        if (isRed) {
            strafeRightTime(speed, time);
        } else {
            strafeLeftTime(speed, time);
        }
        sleep(3);
    }
        /*
     * Code to run ONCE when the driver hits INIT
     */


    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(-leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(-leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }


    public void robotInit() {
        // the extended gamepad object
        pg1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        square2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.X);
        triangle2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.Y);
        x2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.A);
        circle2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.B);
        rightBumper2Reader = new ButtonReader(gp2, GamepadKeys.Button.RIGHT_BUMPER);
        dUp2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.DPAD_UP);
        dDown2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.DPAD_DOWN);
        dLeft2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.DPAD_LEFT);
        dRight2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.DPAD_RIGHT);

        // change the names and directions to suit your robot
        Motor frontLeftMotor = new Motor(hardwareMap, "fl drive");
        Motor frontRightMotor = new Motor(hardwareMap, "fr drive");
        Motor backLeftMotor = new Motor(hardwareMap, "bl drive");
        Motor backRightMotor = new Motor(hardwareMap, "br drive");

        mecanum = new MecanumDrive(backRightMotor, backLeftMotor, frontRightMotor, frontLeftMotor);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fwTopMotor = hardwareMap.get(DcMotorEx.class, "top launcher");
        fwTopMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fwTopMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fwBotMotor = hardwareMap.get(DcMotorEx.class, "bottom launcher ");
        fwBotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fwBotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flipperServo = hardwareMap.get(Servo.class, "flipper");
        indexerServo = hardwareMap.get(Servo.class, "indexer");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
        */
        odo.setOffsets(-84.0, -168., DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
        */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
        */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
        */
        odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }



    /*
     * Code to run ONCE when the driver hits PLAY
     */

    public void noStart() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */


    /*
     * Code to run ONCE after the driver hits STOP
     */

    public void noStop() {
    }

    // **********  HIGH Level driving functions.  ********************

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraightTime(double maxDriveSpeed,
                              long timeMs) {
        long start = System.currentTimeMillis();
        mecanum.driveRobotCentric(0, maxDriveSpeed, 0);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
            sleep(100);
        }

        mecanum.driveRobotCentric(0, 0, 0);
    }

    public void turnTime(double maxTurnSpeed, long timeMs) {
        long start = System.currentTimeMillis();
        mecanum.driveRobotCentric(0, 0, maxTurnSpeed);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
            sleep(100);
            odo.update();
            telemetry.addData("current y: ", odo.getPosY(DistanceUnit.MM));
            telemetry.update();
        }

        mecanum.driveRobotCentric(0, 0, 0);

    }

    public void turnToDegrees(double maxTurnSpeed, double angDeg) {
        odo.resetPosAndIMU();
        sleep(500);
        odo.update();
        if (angDeg > 0) {
            mecanum.driveRobotCentric(0, 0, maxTurnSpeed);
            while (Math.toDegrees(odo.getHeading(AngleUnit.DEGREES)) < angDeg && opModeIsActive()) {
                double degrees = Math.toDegrees(odo.getHeading(AngleUnit.DEGREES));
                telemetry.addData("target:", angDeg);
                telemetry.addData("HR:", odo.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Heading: ", degrees);
                telemetry.addData("pos", odo.getPosition());
                telemetry.addData("pos x", odo.getPosX(DistanceUnit.MM));
                telemetry.addData("pos y", odo.getPosY(DistanceUnit.MM));
                telemetry.update();
                sleep(10);
                odo.update();
            }
        } else {
            mecanum.driveRobotCentric(0, 0, -maxTurnSpeed);
            while (odo.getHeading(AngleUnit.DEGREES) > angDeg && opModeIsActive()) {
                double degrees = Math.toDegrees(odo.getHeading(AngleUnit.DEGREES));
                telemetry.addData("target:", angDeg);
                telemetry.addData("HR:", odo.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Heading: ", degrees);
                telemetry.addData("pos", odo.getPosition());
                telemetry.addData("pos x", odo.getPosX(DistanceUnit.MM));
                telemetry.addData("pos y", odo.getPosY(DistanceUnit.MM));
                telemetry.update();
                sleep(10);
                odo.update();
            }
        }
        mecanum.driveRobotCentric(0, 0, 0);

    }

    public void strafeLeftTime(double maxDriveSpeed,
                           long timeMs) {
        long start = System.currentTimeMillis();
        mecanum.driveRobotCentric(-maxDriveSpeed, 0, 0);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
            sleep(100);
            odo.update();
            telemetry.addData("current y: ", odo.getPosY(DistanceUnit.MM));
            telemetry.update();
        }

        mecanum.driveRobotCentric(0, 0, 0);
    }

    public void strafeLeft(double maxDriveSpeed,
                           double distance) {
        odo.resetPosAndIMU();
        odo.update();
        mecanum.driveRobotCentric(-maxDriveSpeed, 0, 0);
        // Ensure that the OpMode is still active
        while (odo.getPosY(DistanceUnit.MM) < distance && opModeIsActive()) {
            odo.update();
            telemetry.addData("target y: ", distance);
            telemetry.addData("current y: ", odo.getPosY(DistanceUnit.MM));
            telemetry.update();
        }

        mecanum.driveRobotCentric(0, 0, 0);
    }

    public void strafeRight(double maxDriveSpeed,
                           double distance) {
        odo.resetPosAndIMU();
        odo.update();
        mecanum.driveRobotCentric(maxDriveSpeed, 0, 0);
        // Ensure that the OpMode is still active
        while (odo.getPosY(DistanceUnit.MM) > -distance && opModeIsActive()) {
            odo.update();
            telemetry.addData("target y: ", distance);
            telemetry.addData("current y: ", odo.getPosY(DistanceUnit.MM));
            telemetry.update();
        }

        mecanum.driveRobotCentric(0, 0, 0);
    }

    public void strafeRightTime(double maxDriveSpeed, long timeMs) {
        long start = System.currentTimeMillis();
        odo.resetPosAndIMU();
        odo.update();
        mecanum.driveRobotCentric(maxDriveSpeed, 0, 0);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
            sleep(100);
            odo.update();
            telemetry.addData("current y: ", odo.getPosY(DistanceUnit.MM));
            telemetry.update();
        }

        mecanum.driveRobotCentric(0, 0, 0);
    }

}
