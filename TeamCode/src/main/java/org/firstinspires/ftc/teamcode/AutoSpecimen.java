package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@Disabled
@Autonomous(name="Auto Meet 3", group="FTC Lib")
public class AutoSpecimen extends LinearOpMode
{
    // april tag vars
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12; //  this is how close the camera should get to the target (inches)

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

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.

    double oldTime = 0;
    int bucketVerticalPosition = 0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor hMotor, vMotor;
    // input motors exactly as shown below
    MecanumDrive mecanum;
    GamepadEx gpA, gpB;

    long attackStartTime =  0;
    long liftStartTime = 0;
    long odoResetTime = 0;

    Servo bucketServo, ledServo, armServo, pincherServo;
    double pincherPosition = 0;

    boolean iSpecimenMode = false;

    ButtonReader square2ButtonReader, triangle2ButtonReader, circle2ButtonReader, x2ButtonReader, dUp2ButtonReader, dDown2ButtonReader, dLeft2ButtonReader, dRight2ButtonReader, leftStick2ButtonReader, rightStick2ButtonReader;

    double armServoPosition = 0;

    @Override
    public void runOpMode() {
        aprilTagInit();
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


        driveStraight(.3, 500, 0);
        vMotor.setTargetPosition(0);
        vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vMotor.setPower(1);
        sleep(1_000);
        turnToDegrees(.3, -90);
        ledServo.setPosition(0);
        sleep(500);
        ledServo.setPosition(STEMperFiConstants.GB_LED_ORANGE);
        driveStraight(.3, -180, 0);
        sleep(500);
        //strafeLeft(.3, 100);
        strafeLeftTime(.5, 1_500);
        vMotor.setTargetPosition(1);
        vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vMotor.setPower(1);
        sleep(4_000);
        strafeRight(.3, 200);
        vMotor.setTargetPosition(10);
        vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vMotor.setPower(1);
        sleep(1_000);
        moveToAnyTag();
        vMotor.setPower(0);
        turnToDegrees(.3, 75);
        strafeRightTime(0.4, 1_500);

//        strafeLeft(.3, 200);
  //      strafeRight(.3, 200);
        // turnToDegrees(.2, -1.5708);
       // driveStraight(.2, -500, 0);
    }
        /*
     * Code to run ONCE when the driver hits INIT
     */

    public void aprilTagInit() {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftfront_drive");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront_drive");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftback_drive");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback_drive");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");

//        Motor frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
//        Motor frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
//        Motor backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
//        Motor backRight = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips



        if (USE_WEBCAM) {
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }
    }
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
//                .setLensIntrinsics(481.985, 481.985, 334.203, 241.948)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(640,480))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public void moveToAnyTag() {
        double drive = 1;
        while (opModeIsActive() && drive > .1) {
            boolean targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Tag Not Found\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .

            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                moveRobot(drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.

            sleep(10);
        }

    }

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
        gpA = new GamepadEx(gamepad1);
        gpB = new GamepadEx(gamepad2);
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .build();

//        // the extended gamepad object
//        gpA = new GamepadEx(gamepad1);
//        gpB = new GamepadEx(gamepad2);
//
//        square2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.X);
//        triangle2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.Y);
//        x2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.A);
//        circle2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.B);
//
//        dDown2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.DPAD_DOWN);
//        dUp2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.DPAD_UP);
//        dLeft2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.DPAD_LEFT);
//        dRight2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.DPAD_RIGHT);
//        rightStick2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.RIGHT_STICK_BUTTON);
//        leftStick2ButtonReader = new ButtonReader(gpB, GamepadKeys.Button.LEFT_STICK_BUTTON);

        bucketServo = hardwareMap.get(Servo.class, "bucket");
        bucketServo.setPosition(0);
        ledServo = hardwareMap.get(Servo.class, "gbled");
        ledServo.setPosition(STEMperFiConstants.GB_LED_WHITE);
        armServo = hardwareMap.get(Servo.class, "arm");
        armServo.setPosition(armServoPosition);
        pincherServo = hardwareMap.get(Servo.class, "intake");
        pincherServo.setPosition(pincherPosition);

        Motor frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);

        mecanum = new MecanumDrive(backRight, backLeft , frontRight, frontLeft);

        hMotor = hardwareMap.get(DcMotor.class, "h");
        hMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vMotor = hardwareMap.get(DcMotor.class, "v");
        vMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        vMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vMotor.setTargetPosition(0);
        vMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vMotor.setPower(1);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

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

    public void noLoop() {
        odo.update();
        double lx = gpA.getLeftX();
        double ly = gpA.getLeftY();
        double rx = gpA.getRightX();
        if (gamepad1.left_bumper || vMotor.getCurrentPosition() > 400) {
            lx = lx / 2;
            ly = ly / 2;
            rx = rx / 2;
        }
        double degrees = odo.getHeading(AngleUnit.DEGREES);

        mecanum.driveFieldCentric(
                lx,
                ly,
                rx,
                degrees,   // gyro value passed in here must be in degrees
                false
        );

        // Gamepad 1
        if (gamepad1.right_trigger > .9 && gamepad1.left_trigger > .9) {
            long now = System.currentTimeMillis();
            if (now - odoResetTime > 1_000) {
                odo.resetPosAndIMU();
                odoResetTime = now;
                gamepad1.runRumbleEffect(customRumbleEffect);

                ledServo.setPosition(STEMperFiConstants.GB_LED_YELLOW);
            }
        }

        // Gamepad 2
        x2ButtonReader.readValue();
        circle2ButtonReader.readValue();
        square2ButtonReader.readValue();
        triangle2ButtonReader.readValue();

        dUp2ButtonReader.readValue();
        dDown2ButtonReader.readValue();
        dLeft2ButtonReader.readValue();
        dRight2ButtonReader.readValue();
        rightStick2ButtonReader.readValue();
        leftStick2ButtonReader.readValue();

        if (dRight2ButtonReader.wasJustPressed()) {
            this.iSpecimenMode = !this.iSpecimenMode;
            if (this.iSpecimenMode) {

                ledServo.setPosition(STEMperFiConstants.GB_LED_INDIGO);
                hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hMotor.setPower(1);
            } else {
                ledServo.setPosition(STEMperFiConstants.GB_LED_OFF);
            }
        }

        if (iSpecimenMode) {
            if (triangle2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
            } else if (x2ButtonReader.wasJustPressed() || dLeft2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hMotor.setPower(1);
            } else if (circle2ButtonReader.wasJustPressed()) {
                attackStartTime = System.currentTimeMillis();
                ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);
            } else if (square2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                ledServo.setPosition(STEMperFiConstants.GB_LED_ORANGE);
            }
        } else {
            if (triangle2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
            } else if (x2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hMotor.setPower(1);
            } else if (circle2ButtonReader.wasJustPressed()) {
                attackStartTime = System.currentTimeMillis();
                ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);
            } else if (square2ButtonReader.wasJustPressed()) {
                attackStartTime = 0;
                ledServo.setPosition(STEMperFiConstants.GB_LED_ORANGE);
            }
        }
        armServo.setPosition(armServoPosition);
        if (attackStartTime > 0) {
            long attackDuration = System.currentTimeMillis() - attackStartTime;
        }

        pincherServo.setPosition(pincherPosition);




//        telemetry.addData("Left X", lx);
//        telemetry.addData("Left Y", ly);
//        telemetry.addData("Right X", rx);
        telemetry.addData("HR:", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Heading: ", degrees);
        telemetry.addData("pos", odo.getPosition());
        telemetry.addData("pos x", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("pos y", odo.getPosY(DistanceUnit.MM));



        double gp2RY = -gpB.getRightY();
        if (dRight2ButtonReader.wasJustPressed()) {
            hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hMotor.setPower(1);
        } else if (rightStick2ButtonReader.wasJustPressed()) {
            hMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            if (hMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION || (Math.abs(gp2RY) > 0.1)) {
                hMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hMotor.setPower(gp2RY);
            }
        }
        telemetry.addData("targetPosition", bucketVerticalPosition);
        telemetry.addData("vMotor.getTargetPosition)", vMotor.getTargetPosition());
        telemetry.addData("vMotor.getCurrentPosition()", vMotor.getCurrentPosition());

        //vMotor.set(gp2LY);
        telemetry.addData("h", gp2RY);
        telemetry.addData("hMotor.getCurrentPosition()", hMotor.getCurrentPosition());
        telemetry.addData("hMotor.getTargetPosition)", hMotor.getTargetPosition());

    }

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
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {


        odo.resetPosAndIMU();
        odo.update();
        if (distance > 0) {
            mecanum.driveRobotCentric(0, maxDriveSpeed, 0);
            // Ensure that the OpMode is still active
            while (odo.getPosX(DistanceUnit.MM) < distance && opModeIsActive()) {
                odo.update();
                telemetry.addData("target x: ", distance);
                telemetry.addData("current x: ", odo.getPosX(DistanceUnit.MM));
                telemetry.update();
            }
        } else {
            mecanum.driveRobotCentric(0, -maxDriveSpeed, 0);
            // Ensure that the OpMode is still active
            while (odo.getPosX(DistanceUnit.MM) > distance && opModeIsActive()) {
                odo.update();
                telemetry.addData("target x: ", distance);
                telemetry.addData("current x: ", odo.getPosX(DistanceUnit.MM));
                telemetry.update();
            }
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
                ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
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
                ledServo.setPosition(STEMperFiConstants.GB_LED_BLUE);
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
        ledServo.setPosition(STEMperFiConstants.GB_LED_OFF);
        mecanum.driveRobotCentric(0, 0, 0);

    }

    public void strafeLeftTime(double maxDriveSpeed,
                           long timeMs) {
        long start = System.currentTimeMillis();
        odo.resetPosAndIMU();
        odo.update();
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
