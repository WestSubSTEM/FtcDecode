package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public Servo flipperServo, indexerServo, turretServo;
    public double flipperServoPosition = STEMperFiConstants.FLIPPER_INTAKE;
    public double indexerServoPosition = STEMperFiConstants.INDEX_1;
    public boolean intakeOn = false;
    public long servoIndexPressTimeMS = 0;
    public long odoResetTimeMS = 0;
    public double shooterSpeed = 0;
    public double turretPosition = STEMperFiConstants.TURRET_CENTER;
    public boolean shooterTriggerPressed = false;
    public long now = System.currentTimeMillis();
    public List<LynxModule> hubs;
    private Gamepad gamepad1, gamepad2;
    private long indexDelayDueToShooting = 0;
    public Limelight3A limelight;

    public long lastDetect = 0;

    private Telemetry telemetry;
    public void init(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

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
        turretServo = hardwareMap.get(Servo.class, "turret");
        turretServo.setPosition(turretPosition);

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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
        double newPosition = shooterTriggerPressed ? STEMperFiConstants.FLIPPER_SHOOT : STEMperFiConstants.FLIPPER_INTAKE;
        if (newPosition != flipperServoPosition) {
            indexDelayDueToShooting = now + STEMperFiConstants.SHOOT_DELAY_INDEX_MS;
        }
        flipperServoPosition = newPosition;
        flipperServo.setPosition(flipperServoPosition);
    }

    public void indexer(boolean ledStatus) {
        // INDEXER
        if (!shooterTriggerPressed && now > indexDelayDueToShooting) {
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                indexerServoPosition = STEMperFiConstants.INDEX_2;
                servoIndexPressTimeMS = now;
                if (ledStatus) {
                    //ledServo.setPosition(STEMperFiConstants.GB_LED_BLUE);
                }
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                indexerServoPosition = STEMperFiConstants.INDEX_1;
                servoIndexPressTimeMS = now;
                if (ledStatus) {
                    //ledServo.setPosition(STEMperFiConstants.GB_LED_RED);
                }
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                indexerServoPosition = STEMperFiConstants.INDEX_3;
                servoIndexPressTimeMS = now;
                if (ledStatus) {
                    //ledServo.setPosition(STEMperFiConstants.GB_LED_GREEN);
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
        if (intakeOn || shooterTriggerPressed || ((now - servoIndexPressTimeMS) < STEMperFiConstants.INTAKE_DURING_INDEX_MOVE_MS)) {
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
            shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_MED;
        } else if (gp2.getButton(GamepadKeys.Button.B)) { // CIRCLE
            shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_HIGH;
        }
        telemetry.addData("shooterSpeed", shooterSpeed);
        if (shooterSpeed == 0) {
            fwTopMotor.stopMotor();
            fwBotMotor.stopMotor();
        } else {
            fwTopMotor.set(shooterSpeed);
            fwBotMotor.set(shooterSpeed);
        }
        double getPower = fwBotMotor.motor.getPower();
        if (fwBotMotor.motor.getPower() > .8) {
            fwBotMotor.motor.setPower(.8);
            fwTopMotor.motor.setPower(.8);
        }
        telemetry.addData("getPower", getPower);
        telemetry.update();
    }
    public void manualTurret () {
        if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            turretPosition += 0.1;
        } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            turretPosition += 0.001;
        } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            turretPosition += -0.1;
        } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            turretPosition += -0.001;
        }
        if (turretPosition>1){
            turretPosition=1;
        } else if (turretPosition<0) {
            turretPosition=0;
        }
        turretServo.setPosition(turretPosition);
    }

    public void calibrateTurret () {
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            turretPosition = 0.2;
        } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            turretPosition = 0.7;
        }
        turretServo.setPosition(turretPosition);
    }

    long nextTurretUpdate = 0;
    double ratio = 0;
    double turretNudge = 0.001;
    public void adjustTurret(double xdif) {
        if (nextTurretUpdate < now) {
            double degreeLimit = 3.0;
            double pixelDiff = 25;
            double pixelCenter = 635;
            double pixelDifCenter = xdif - pixelCenter;
            //if (xdif > degreeLimit) {
            ratio = pixelDifCenter / pixelDiff;
            telemetry.addData("ratio", ratio);
            if (Math.abs(ratio) > 1) {
                telemetry.addData("turretNudge", turretNudge * ratio);
                turretPosition += turretNudge * ratio;
                nextTurretUpdate = (long) (now + (10 * ( (turretNudge * Math.abs(ratio)) / STEMperFiConstants.SERVO_TRAVEL_PER_MS)));
            } else {
                telemetry.addData("turretNudge", 0);
            }
            turretPosition = Math.min(turretPosition, .8);
            turretPosition = Math.max(turretPosition, .2);
            turretServo.setPosition(turretPosition);
        } else {
            telemetry.addData("ratio", ratio);
            telemetry.addData("turretNudge wait", ratio * turretNudge);
            telemetry.addData("turretNudge ms", (long) (10 * ( (turretNudge * ratio) / STEMperFiConstants.SERVO_TRAVEL_PER_MS)));
        }
        telemetry.addData("turretPosition", turretPosition);
    }

    public void detect() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            LLResultTypes.FiducialResult bob = fiducialResults.get(0);
            if (bob != null) {
                lastDetect = now;
                double xDif = bob.getTargetXDegrees();
                double xPixDif = bob.getTargetXPixels();
                double xNoCrossDif = bob.getTargetXDegreesNoCrosshair();
                telemetry.addData("Fiducial", "ID: %d, XDeg: %.1f, Xpix: %.1f", bob.getFiducialId(), xDif, xPixDif);
                telemetry.addData("Fiducial", "ID: %d, XnoC: %.1f, Xpix: %.1f", bob.getFiducialId(), xNoCrossDif, xPixDif);
                adjustTurret(xPixDif);
            }
            telemetry.addData("turret", turretPosition);
            telemetry.update();
        }
        if (now - lastDetect > 1_000) {
            turretPosition = STEMperFiConstants.TURRET_CENTER;
            turretServo.setPosition(turretPosition);
        }

    }
}
