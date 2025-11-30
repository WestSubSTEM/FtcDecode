package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Configurable
@TeleOp(name = "FlyWheel", group = "Meet2")
public class FlyWheelTest extends LinearOpMode  {
    public static double SET_POWER_VALUE = 0.0;
    public static double calculated = 0.1833333333;

    public static int testInt = 42;


    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
    DcMotorEx intakeMotor;
    MotorEx fwTopMotor, fwBotMotor;

    // input motors exactly as shown below
    MecanumDrive mecanum;
    GamepadEx pg1, gp2;
    ButtonReader square2ButtonReader, triangle2ButtonReader, circle2ButtonReader, x2ButtonReader, rightBumper2Reader, dUp2ButtonReader, dDown2ButtonReader, dLeft2ButtonReader, dRight2ButtonReader, leftStick2ButtonReader, rightStick2ButtonReader;
    Motor fwTop, fwBot;
    Servo flipperServo, indexerServo;
    double flipperServoPosition = STEMperFiConstants.FLIPPER_INTAKE;
    double indexerServoPosition = STEMperFiConstants.INDEX_1;
    boolean intakeOn = false;
    long serverIndexPressTimeMS = 0;

    TelemetryManager t = PanelsTelemetry.INSTANCE.getTelemetry();

    private GamepadEx toolOp;


    public static double kP = 20;
    public static double kV = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        toolOp = new GamepadEx(gamepad2);

        // this creates a group of two 6k RPM goBILDA motors
        // the 'flywheel_left' motor in the configuration will be set
        // as the leader for the group
        fwTop = new Motor(hardwareMap, "top launcher", Motor.GoBILDA.BARE);
        fwBot = new Motor(hardwareMap, "bottom launcher ", Motor.GoBILDA.BARE);

        fwTop.setRunMode(Motor.RunMode.VelocityControl);
        fwTop.setVeloCoefficients(kP, 0, 0);
        fwTop.setFeedforwardCoefficients(0, kV);

        fwBot.setRunMode(Motor.RunMode.VelocityControl);
        fwBot.setVeloCoefficients(kP, 0, 0);
        fwBot.setFeedforwardCoefficients(0, kV);

        // this is not required for this example
        // here, we are setting the bulk caching mode to manual so all hardware reads
        // for the motors can be read in one hardware call.
        // we do this in order to decrease our loop time
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            // This clears the cache for the hardware
            // Refer to https://gm0.org/en/latest/docs/software/control-system-internals.html#bulk-reads
            // for more information on bulk reads.
            hubs.forEach(LynxModule::clearBulkCache);


            if (toolOp.wasJustPressed(GamepadKeys.Button.A)) {
                SET_POWER_VALUE = SET_POWER_VALUE + 0.05;
                SET_POWER_VALUE = SET_POWER_VALUE > 1 ? 1 : SET_POWER_VALUE;
            } else if (toolOp.wasJustPressed(GamepadKeys.Button.B)) {
                SET_POWER_VALUE = SET_POWER_VALUE - 0.05;
                SET_POWER_VALUE = SET_POWER_VALUE < 0 ? 0 : SET_POWER_VALUE;
            } else if (toolOp.wasJustPressed(GamepadKeys.Button.X)) {
                SET_POWER_VALUE = calculated;
            } else if (toolOp.wasJustPressed(GamepadKeys.Button.Y)) {
                SET_POWER_VALUE = 0;
            }
            if (SET_POWER_VALUE == 0) {
                fwTop.stopMotor();
                fwBot.stopMotor();
            } else {
                fwTop.set(SET_POWER_VALUE);
                if (fwTop.motor.getPower() > .8) {
                    fwTop.motor.setPower(0.8);
                }
                fwBot.set(SET_POWER_VALUE);
                if (fwBot.motor.getPower() > .8) {
                    fwBot.motor.setPower(0.8);
                }
            }

            // we can obtain a list of velocities with each item in the list
            // representing the motor passed in as an input to the constructor.
            // so, our flywheel_left is index 0 and flywheel_right is index 1
            t.debug("top.getMaxRPM", fwTop.getMaxRPM());
            t.debug("top.get", fwTop.get());
            t.debug("top.ACHIEVABLE_MAX_TICKS_PER_SECOND", fwTop.ACHIEVABLE_MAX_TICKS_PER_SECOND);
            t.debug("top.getRate", fwTop.getRate());
            t.debug("top.rate_div_max",  fwTop.getRate() / fwTop.ACHIEVABLE_MAX_TICKS_PER_SECOND);
            t.debug("top.getCPR", fwTop.getCPR());
            t.debug("SET_POWER_VALUE", SET_POWER_VALUE);
            t.debug("testInt", testInt);
            t.update(telemetry);
            toolOp.readButtons();
        }
    }

}