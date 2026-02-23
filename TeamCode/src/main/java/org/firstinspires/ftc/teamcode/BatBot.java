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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;

import java.util.Arrays;
import java.util.List;

@Configurable
public class BatBot
{
    NormalizedColorSensor cs3, cs2;
    public List<Color> indexerContents = Arrays.asList(Color.WHITE, Color.WHITE, Color.WHITE);

    public GoBildaPrismDriver prism;
    public static volatile double FLYWHEEL_kP = 20.0;
    public static volatile double FLYWHEEL_kV = 00.7;
    public boolean isRed = true;
    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    public Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
    public DcMotor intakeMotor;
    public Motor fwBotMotor;
    public DcMotorEx turretMotor;
    public int turretTargetPosition = 0;
    // input motors exactly as shown below
    public MecanumDrive mecanum;
    public GamepadEx gp1, gp2;
//    public ButtonReader square2ButtonReader, triangle2ButtonReader, circle2ButtonReader, x2ButtonReader, rightBumper2Reader, dUp2ButtonReader, dDown2ButtonReader, dLeft2ButtonReader, dRight2ButtonReader, leftStick2ButtonReader, rightStick2ButtonReader;
    public Servo flipperServo;
    public Servo indexerServo, indexerLed, hoodServo;
    public double hoodPosition = 1.0;
    public double flipperServoPosition = STEMperFiConstants.FLIPPER_INTAKE;
    public double indexerServoPosition = STEMperFiConstants.INDEX_1;
    public boolean intakeOn = false;
    public long indexerTimePressed = 0;
    public int indexerIndex = 0;
    public long odoResetTimeMS = 0;
    public double shooterSpeed = 0;
    public boolean shooterTriggerPressed = false;
    public long now = System.currentTimeMillis();
    public List<LynxModule> hubs;
    public Gamepad gamepad1, gamepad2;
    public long indexDelayDueToShooting = 0;
    public Limelight3A limelight;
    public long lastDetect = 0;
    public String pattern = STEMperFiConstants.PATTERN_21_GPP;

    public boolean indexMoved = false;

    public NormalizedRGBA cs2RgbaBase, cs3RgbaBase;

    public Telemetry telemetry;
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

        /*
         * Initialize the hardware variables. Note that the strings used here must correspond
         * to the names assigned during the robot configuration step on the driver's station.
         */
        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");

        /*
         * Set the number of LEDs (starting at 0) that are in your strip. This can be longer
         * than the actual length of the strip, but some animations won't look quite right.
         */
        prism.setStripLength(STEMperFiConstants.LED_NUM);



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

        fwBotMotor = new Motor(hardwareMap, "bottom launcher", Motor.GoBILDA.BARE);
        fwBotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fwBotMotor.setRunMode(MotorEx.RunMode.VelocityControl);
        fwBotMotor.setVeloCoefficients(FLYWHEEL_kP, 0, 0);
        fwBotMotor.setFeedforwardCoefficients(0, FLYWHEEL_kV);

        hoodServo =  hardwareMap.get(Servo.class, "hood");

        flipperServo = hardwareMap.get(Servo.class, "flipper");
        indexerServo = hardwareMap.get(Servo.class, "indexer");
        indexerLed = hardwareMap.get(Servo.class, "indexerLed");

        turretMotor = hardwareMap.get(DcMotorEx.class, "lazy");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cs2 = hardwareMap.get(NormalizedColorSensor.class, "color_range");
        cs3 = hardwareMap.get(NormalizedColorSensor.class, "color_v3");
        if (cs3 instanceof SwitchableLight) {
            ((SwitchableLight)cs3).enableLight(true);
        }
        if (cs2 instanceof SwitchableLight) {
            ((SwitchableLight)cs2).enableLight(true);
        }
        cs2.setGain(STEMperFiConstants.COLOR_SENSOR_GAIN);
        cs3.setGain(STEMperFiConstants.COLOR_SENSOR_GAIN);

        cs2RgbaBase = cs2.getNormalizedColors();
        cs3RgbaBase = cs3.getNormalizedColors();

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
    public void setIndexerPosition(int index) {

        if (index >= 0 && index <=3) {
            double indexerServoPositionNew = STEMperFiConstants.INDEXES.get(index);
            if (indexerServoPositionNew != indexerServoPosition) {
                indexerServoPosition = indexerServoPositionNew;
                indexerServo.setPosition(indexerServoPosition);
                indexerTimePressed = now;
                indexerIndex = index;
                indexMoved = true;
//                Color currentColor = indexerContents.get(indexerIndex);
//                if (currentColor == Color.WHITE) {
//                    indexerLed.setPosition(STEMperFiConstants.GB_LED_WHITE);
//                } else if (currentColor == Color.PURPLE) {
//                    indexerLed.setPosition(STEMperFiConstants.GB_LED_VIOLET);
//                } else {
//                    indexerLed.setPosition(STEMperFiConstants.GB_LED_GREEN);
//                }
            }
        }
    }

    public void adjustHood() {
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            hoodPosition += .1;
        } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            hoodPosition -= .1;
        } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            hoodPosition += .01;
        } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            hoodPosition -= .01;
        }
        if (hoodPosition > 1) {
            hoodPosition = 1;
        } else if (hoodPosition < 0) {
            hoodPosition = 0;
        }
        //telemetry.addData("hood:", hoodPosition);
        hoodServo.setPosition(hoodPosition);
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

    public void indexer(boolean isInit) {
        // INDEXER
        if (!shooterTriggerPressed && now > indexDelayDueToShooting) {
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                setIndexerPosition(1);
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                setIndexerPosition(0);
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                setIndexerPosition(2);
            }
        }
    }
    private double percentDifference(double newValue, double oldValue) {
      return oldValue != 0 ? 100 * (newValue - oldValue) / oldValue : 0;
    }
    public void intake() {
        if (isIndexerFull()) {
            intakeOn = false;
        } else if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            intakeOn = !intakeOn;
        }

        double leftTrigger = gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double intakePower = 0;
        if (intakeOn || shooterTriggerPressed || ((now - indexerTimePressed) < STEMperFiConstants.INTAKE_DURING_INDEXER_MOVE_MS)) {
            intakePower = 1;
        } else if (leftTrigger > 0.2) {
            intakePower = -leftTrigger;
        }
        intakeMotor.setPower(intakePower);

//        if (indexerContents.get(indexerIndex) == Color.WHITE && isBallIn()) {
//            Color ballColor = determineColor();
//            indexerContents.set(indexerIndex, ballColor);
//            if (isIndexerFull()) {
//                setIndexerPosition(2);
//            }
//        }

    }
    public Color determineColor(){
        //Evaluate color
        Color answer = Color.WHITE;
        //Read the color sensors
        NormalizedRGBA cs2RgbaNew = cs2.getNormalizedColors();
        NormalizedRGBA cs3RgbaNew = cs3.getNormalizedColors();

        telemetry.addLine("cs2.toColor: " + cs2RgbaNew.toColor() + ", cs2.a: " + cs2RgbaNew.alpha + ", cs2.r: " + cs2RgbaNew.red + ", cs2.g: " + cs2RgbaNew.green + ", cs2.b: " + cs2RgbaNew.blue);
        telemetry.addLine("cs3.toColor: " + cs3RgbaNew.toColor() + ", cs3.a: " + cs3RgbaNew.alpha + ", cs3.r: " + cs3RgbaNew.red + ", cs3.g: " + cs3RgbaNew.green + ", cs3.b: " + cs3RgbaNew.blue);
//        double cs2PercentDiffRed = percentDifference(cs2RgbaNew.red, cs2RgbaBase.red);
//        double cs2PercentDiffGreen = percentDifference(cs2RgbaNew.green, cs2RgbaBase.green);
//        double cs2PercentDiffBlue = percentDifference(cs2RgbaNew.blue, cs2RgbaBase.blue);
//        double cs3PercentDiffBlue = percentDifference(cs3RgbaNew.blue, cs3RgbaBase.blue);
//        // Green
//        int greenPoints = 0;
//        if (cs2PercentDiffGreen > cs2PercentDiffBlue && cs2PercentDiffGreen > cs2PercentDiffRed && cs2PercentDiffGreen > 80) {
//            greenPoints++;
//        }
//        if (cs3PercentDiffBlue > 100 && cs3PercentDiffBlue < 160) {
//            greenPoints++;
//        }
//        // purple
//        int purplePoints = 0;
//        if (cs2PercentDiffGreen < cs2PercentDiffBlue && cs2PercentDiffGreen < cs2PercentDiffRed && cs2PercentDiffRed > 80 && cs2PercentDiffBlue > 80) {
//            purplePoints++;
//        }
//        if (cs2PercentDiffBlue > 160) {
//            purplePoints++;
//        }
//        if (greenPoints > 0 || purplePoints > 0) {
//            answer = greenPoints > purplePoints ? Color.GREEN : Color.PURPLE;
//        }
        return answer;
    }

//    public Color getCurrentColor() {
//        return indexerContents.get(indexerIndex);
//    }
//
    public boolean isBallIn(){
        return ((DistanceSensor) cs2).getDistance(DistanceUnit.CM) <STEMperFiConstants.BALL_DETECTION_DISTANCE_CM;
    }

    public boolean isIndexerFull() {
        //return !indexerContents.contains(Color.WHITE);
        return false;
    }

    public boolean moveToColor(Color ballColor){
        int ballColorIndex=indexerContents.indexOf(ballColor);
        if (ballColorIndex==-1){
            return false;
        }
        setIndexerPosition(ballColorIndex);
        return true;
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
            hoodPosition = STEMperFiConstants.HOOD_RELATIVE_ANGLE_SHORT;
        } else if (gp2.getButton(GamepadKeys.Button.Y)) { // TRIANGLE
            shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_MED;
            hoodPosition = STEMperFiConstants.HOOD_RELATIVE_ANGLE_MED;
        } else if (gp2.getButton(GamepadKeys.Button.B)) { // CIRCLE
            shooterSpeed = STEMperFiConstants.SHOOT_RELATIVE_POWER_HIGH;
        }
        hoodServo.setPosition(hoodPosition);
        //telemetry.addData("shooterSpeed", shooterSpeed);
        if (shooterSpeed == 0) {
            fwBotMotor.stopMotor();
        } else {
            fwBotMotor.set(shooterSpeed);
        }
        double getPower = fwBotMotor.motor.getPower();
        if (fwBotMotor.motor.getPower() > .8) {
            fwBotMotor.motor.setPower(.8);
        }
        //telemetry.addData("getPower", getPower);
        //telemetry.update();
    }
//    public void manualTurret () {
//        turretMotor.setRunMode(Motor.RunMode.RawPower);
//        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//        if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
//            turretMotor.set(turretMotor.get() + .1);
//        } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//            turretMotor.set(turretMotor.get() + .01);
//        } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
//            turretMotor.set(turretMotor.get() - .1);
//        } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//            turretMotor.set(turretMotor.get() - .01);
//        } else if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
//            turretMotor.set(0);
//        }
//        telemetry.addData("turret pow: ", turretMotor.get());
//        telemetry.addData("turret pos: ", turretMotor.getCurrentPosition());
//        telemetry.addData("turret ang", turretMotor.getCurrentPosition()/ STEMperFiConstants.TURRET_TICKS_PER_DEGREE );
//    }

    public void calibrateTurret () {
        if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            turretTargetPosition += 1_000 * 5;
        } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            turretTargetPosition += 1_000;
        } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            turretTargetPosition -= 1_000 * 5;
        } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            turretTargetPosition -= 1_000;
        } else if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            turretTargetPosition = -turretMotor.getCurrentPosition();
        } else if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            turretTargetPosition = 0;
        }
        turretTargetPosition = Math.min(turretTargetPosition, STEMperFiConstants.TURRET_MAX_TICKS);
        turretTargetPosition = Math.max(turretTargetPosition, -STEMperFiConstants.TURRET_MAX_TICKS);
        //telemetry.addData("turretTargetPosition:",turretTargetPosition);
   }

    long nextTurretUpdate = 0;

    public void adjustTurretTargetPosition(double xdif) {
        int currentPosition = -turretMotor.getCurrentPosition();
        int newPosition = currentPosition + (int) (xdif * STEMperFiConstants.TURRET_TICKS_PER_DEGREE);
        if (newPosition > STEMperFiConstants.TURRET_MAX_TICKS) {
            newPosition = STEMperFiConstants.TURRET_MAX_TICKS;
        } else if (newPosition < STEMperFiConstants.TURRET_MIN_TICKS) {
            newPosition = STEMperFiConstants.TURRET_MIN_TICKS;
        }
        //telemetry.addData("turret    cur pos", currentPosition);
        //telemetry.addData("turret target pos", newPosition);
        turretTargetPosition = newPosition;
    }

    public boolean setTurretPower() {
        int currentPosition = -turretMotor.getCurrentPosition();
        int dif = turretTargetPosition - currentPosition;
        isOnTarget = Math.abs(dif) < STEMperFiConstants.TURRET_TARGET_DELTA;
        double newTurretPower = STEMperFiConstants.TURRET_MOTOR_POWER_MAX * 1.8 * (((double)dif) / (double)STEMperFiConstants.TURRET_MAX_TICKS);
        newTurretPower = Math.min(STEMperFiConstants.TURRET_MOTOR_POWER_MAX, newTurretPower);
        newTurretPower = Math.max(-STEMperFiConstants.TURRET_MOTOR_POWER_MAX, newTurretPower);
        newTurretPower = newTurretPower > 0 && newTurretPower < STEMperFiConstants.TURRET_MOTOR_POWER_MIN ? STEMperFiConstants.TURRET_MOTOR_POWER_MIN : newTurretPower;
        newTurretPower = newTurretPower < 0 && newTurretPower > -STEMperFiConstants.TURRET_MOTOR_POWER_MIN ? -STEMperFiConstants.TURRET_MOTOR_POWER_MIN : newTurretPower;
        if (isOnTarget) {
            newTurretPower = 0;
        }
        turretMotor.setPower(newTurretPower);
        //telemetry.addData("current  Pos: ", currentPosition);
        //telemetry.addData("turret Power: ", newTurretPower);
        return isOnTarget;
    }

    boolean isOnTarget = false;
    public boolean detectGoal(long timeout) {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            LLResultTypes.FiducialResult fiducialResult = fiducialResults.get(0);
            if (fiducialResult != null) {
                lastDetect = now;
                double xDif = -fiducialResult.getTargetXDegrees();
                double xPixDif = fiducialResult.getTargetXPixels();
                double xNoCrossDif = fiducialResult.getTargetXDegreesNoCrosshair();
                //telemetry.addData("Fiducial", "ID: %d, XDeg: %.1f, Xpix: %.1f", fiducialResult.getFiducialId(), xDif, xPixDif);
                //telemetry.addData("Fiducial", "ID: %d, XnoC: %.1f, Xpix: %.1f", fiducialResult.getFiducialId(), xNoCrossDif, xPixDif);
                adjustTurretTargetPosition(xDif);
                return setTurretPower();
            }
        } else if (timeout > 0 && now - lastDetect > timeout) {
            //telemetry.addLine("April Tag not detected.");
            //telemetry.addLine("April Tag not detected.");
            turretTargetPosition = 0;
            return setTurretPower();
        }
        return false;
    }

    public boolean detectAutoPattern() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            LLResultTypes.FiducialResult fiducialResult = fiducialResults.get(0);
            if (fiducialResult != null) {
                switch (fiducialResult.getFiducialId()) {
                    case 21:
                        pattern = STEMperFiConstants.PATTERN_21_GPP;
                        break;
                    case 22:
                        pattern = STEMperFiConstants.PATTERN_22_PGP;
                        break;
                    case 23:
                        pattern = STEMperFiConstants.PATTERN_23_PPG;
                        break;
                    default:
                        return false;
                }
                return true;
            }
        }
        return false;
    }

    public void setAllLedsSolid(Color color) {
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, STEMperFiConstants.getAnimationSolid(color, 0, STEMperFiConstants.LED_NUM - 1, STEMperFiConstants.LED_BRIGHTNESS));
    }
}
