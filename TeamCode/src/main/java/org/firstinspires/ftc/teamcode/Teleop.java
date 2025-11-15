package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Tele Meet 1", group = "Meet1")
public class Teleop extends OpMode {
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

    @Override
    public void init() {
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
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        odo.resetPosAndIMU();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        odo.update();
        double lx = pg1.getLeftX();
        double ly = pg1.getLeftY();
        double rx = pg1.getRightX();
        if (gamepad1.left_bumper) {
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


        // Gamepad 2
        x2ButtonReader.readValue();
        circle2ButtonReader.readValue();
        square2ButtonReader.readValue();
        triangle2ButtonReader.readValue();
        rightBumper2Reader.readValue();

        dUp2ButtonReader.readValue();
        dDown2ButtonReader.readValue();
        dLeft2ButtonReader.readValue();
        dRight2ButtonReader.readValue();

        // INDEXER
        long now = System.currentTimeMillis();
        if (dUp2ButtonReader.wasJustPressed()) {
            indexerServoPosition = STEMperFiConstants.INDEX_2;
            serverIndexPressTimeMS = now;
        } else if (dLeft2ButtonReader.wasJustPressed()) {
            indexerServoPosition = STEMperFiConstants.INDEX_1;
            serverIndexPressTimeMS = now;
        } else if (dRight2ButtonReader.wasJustPressed()) {
            indexerServoPosition = STEMperFiConstants.INDEX_3;
            serverIndexPressTimeMS = now;
        }
        indexerServo.setPosition(indexerServoPosition);


        // SHOOTER
        boolean shootButtonPressed = gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2;
        flipperServoPosition = shootButtonPressed ? STEMperFiConstants.FLIPPER_SHOOT : STEMperFiConstants.FLIPPER_INTAKE;
        flipperServo.setPosition(flipperServoPosition);

        // INTAKE
        if (rightBumper2Reader.wasJustPressed()) {
            intakeOn = !intakeOn;
        }

        double leftTrigger = gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double intakePower = 0;
        if (intakeOn || shootButtonPressed || ((now - serverIndexPressTimeMS) < STEMperFiConstants.INTAKE_DURING_INDEX_MOVE_MS)) {
            intakePower = 1;
        } else if (leftTrigger > 0.2) {
            intakePower = -leftTrigger;
        }
        intakeMotor.setPower(intakePower);


        // FLYWHEEL
        if (x2ButtonReader.isDown()) {
            fwTopMotor.setPower(0);
            fwBotMotor.setPower(0);
        } else if (square2ButtonReader.isDown()) {
            fwTopMotor.setPower(.33);
            fwBotMotor.setPower(.33);
        } else if (triangle2ButtonReader.isDown()) {
            fwTopMotor.setPower(.66);
            fwBotMotor.setPower(.66);
        } else if (circle2ButtonReader.isDown()) {
            fwTopMotor.setPower(1);
            fwBotMotor.setPower(1);
        }
    }
}