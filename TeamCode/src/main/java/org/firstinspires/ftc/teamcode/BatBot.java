package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Configurable
public class BatBot
{
    public static volatile double FLYWHEEL_kP = 20.0;
    public static volatile double FLYWHEEL_kV = 00.7;

    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    public Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
    public DcMotor intakeMotor;
    public Motor fwTopMotor, fwBotMotor;
    // input motors exactly as shown below
    public MecanumDrive mecanum;
    public GamepadEx gp1, gp2;
//    public ButtonReader square2ButtonReader, triangle2ButtonReader, circle2ButtonReader, x2ButtonReader, rightBumper2Reader, dUp2ButtonReader, dDown2ButtonReader, dLeft2ButtonReader, dRight2ButtonReader, leftStick2ButtonReader, rightStick2ButtonReader;
    public Servo flipperServo, indexerServo, ledServo;
    public double flipperServoPosition = STEMperFiConstants.FLIPPER_INTAKE;
    public double indexerServoPosition = STEMperFiConstants.INDEX_1;
    public boolean intakeOn = false;
    public long serverIndexPressTimeMS = 0;
    public long odoResetTimeMS = 0;
    public double shooterSpeed = 0;
    public boolean shooterTriggerPressed = false;
    public long now = System.currentTimeMillis();
    public List<LynxModule> hubs;
    private Gamepad gamepad1, gamepad2;
    public void init(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .build();

        // this is not required for this example
        // here, we are setting the bulk caching mode to manual so all hardware reads
        // for the motors can be read in one hardware call.
        // we do this in order to decrease our loop time
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        // the extended gamepad object
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

//        square2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.X);
//        triangle2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.Y);
//        x2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.A);
//        circle2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.B);
//        rightBumper2Reader = new ButtonReader(gp2, GamepadKeys.Button.RIGHT_BUMPER);
//        dUp2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.DPAD_UP);
//        dDown2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.DPAD_DOWN);
//        dLeft2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.DPAD_LEFT);
//        dRight2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.DPAD_RIGHT);

        // change the names and directions to suit your robot
        Motor frontLeftMotor = new Motor(hardwareMap, "fl drive");
        Motor frontRightMotor = new Motor(hardwareMap, "fr drive");
        Motor backLeftMotor = new Motor(hardwareMap, "bl drive");
        Motor backRightMotor = new Motor(hardwareMap, "br drive");

        mecanum = new MecanumDrive(backRightMotor, backLeftMotor, frontRightMotor, frontLeftMotor);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fwTopMotor = new Motor(hardwareMap, "top launcher", Motor.GoBILDA.BARE);
        fwTopMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fwTopMotor.setRunMode(MotorEx.RunMode.VelocityControl);
        fwTopMotor.setVeloCoefficients(FLYWHEEL_kP, 0, 0);
        fwTopMotor.setFeedforwardCoefficients(0, FLYWHEEL_kV);
        fwBotMotor = new Motor(hardwareMap, "bottom launcher", Motor.GoBILDA.BARE);
        fwBotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fwBotMotor.setRunMode(MotorEx.RunMode.VelocityControl);
        fwBotMotor.setVeloCoefficients(FLYWHEEL_kP, 0, 0);
        fwBotMotor.setFeedforwardCoefficients(0, FLYWHEEL_kV);

        flipperServo = hardwareMap.get(Servo.class, "flipper");
        indexerServo = hardwareMap.get(Servo.class, "indexer");
        ledServo = hardwareMap.get(Servo.class, "gbled");
        ledServo.setPosition(STEMperFiConstants.GB_LED_WHITE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
        */
        // MEET 1 Values odo.setOffsets(-84.0, -168., DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setOffsets(157, 80., DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
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

        // DO this as needed in each
        //odo.recalibrateIMU();
        //odo.resetPosAndIMU();
    }

    public void mecanumDrive() {
        odo.update();
        double lx = gp1.getLeftX();
        double ly = gp1.getLeftY();
        double rx = gp1.getRightX();
        if (gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
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
        if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .9 &&gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .9) {
            if (now - odoResetTimeMS > 1_000) {
                odo.resetPosAndIMU();
                odoResetTimeMS = now;
                gamepad1.runRumbleEffect(customRumbleEffect);

                ledServo.setPosition(STEMperFiConstants.GB_LED_YELLOW);
            }
        }
    }

    public void startLoop() {
        // This clears the cache for the hardware
        // Refer to https://gm0.org/en/latest/docs/software/control-system-internals.html#bulk-reads
        // for more information on bulk reads.
        hubs.forEach(LynxModule::clearBulkCache);

        now = System.currentTimeMillis();
        gp1.readButtons();
        gp2.readButtons();
        shooterTriggerPressed = gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2;
    }

    public void shoot() {
        flipperServoPosition = shooterTriggerPressed ? STEMperFiConstants.FLIPPER_SHOOT : STEMperFiConstants.FLIPPER_INTAKE;
        flipperServo.setPosition(flipperServoPosition);
    }

    public void indexer(boolean ledStatus) {
        // INDEXER
        if (!shooterTriggerPressed) {
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                indexerServoPosition = STEMperFiConstants.INDEX_2;
                serverIndexPressTimeMS = now;
                if (ledStatus) {
                    ledServo.setPosition(STEMperFiConstants.GB_LED_BLUE);
                }
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                indexerServoPosition = STEMperFiConstants.INDEX_1;
                serverIndexPressTimeMS = now;
                if (ledStatus) {
                    ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
                }
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                indexerServoPosition = STEMperFiConstants.INDEX_3;
                serverIndexPressTimeMS = now;
                if (ledStatus) {
                    ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);
                }
            }
            indexerServo.setPosition(indexerServoPosition);
        }
    }

    public void intake() {
        if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            intakeOn = !intakeOn;
        }

        double leftTrigger = gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double intakePower = 0;
        if (intakeOn || shooterTriggerPressed || ((now - serverIndexPressTimeMS) < STEMperFiConstants.INTAKE_DURING_INDEX_MOVE_MS)) {
            intakePower = 1;
        } else if (leftTrigger > 0.2) {
            intakePower = -leftTrigger;
        }
        intakeMotor.setPower(intakePower);
    }

    public void flywheel() {
//        square2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.X);
//        triangle2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.Y);
//        x2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.A);
//        circle2ButtonReader = new ButtonReader(gp2, GamepadKeys.Button.B);
        if (gp2.getButton(GamepadKeys.Button.A)) { // CROSS
            shooterSpeed = 0;
        } else if (gp2.getButton(GamepadKeys.Button.X)) { // SQUARE
            shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_SHORT;
        } else if (gp2.getButton(GamepadKeys.Button.Y)) { // TRIANGLE
            shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_SHORT;
        } else if (gp2.getButton(GamepadKeys.Button.B)) { // CIRCLE
            shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_SHORT;
        }
        if (shooterSpeed == 0) {
            fwTopMotor.stopMotor();
            fwBotMotor.stopMotor();
        } else {
            fwTopMotor.set(shooterSpeed);
            fwBotMotor.set(shooterSpeed);
        }
        if (fwBotMotor.motor.getPower() > .8) {
            fwBotMotor.motor.setPower(.8);
            fwTopMotor.motor.setPower(.8);
        }
    }
}
