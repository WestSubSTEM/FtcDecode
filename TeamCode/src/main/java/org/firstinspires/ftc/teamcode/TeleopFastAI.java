package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.Prism.Color;

@TeleOp(name = "Tele QUAL AI", group = "COMP")
public class TeleopFastAI extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    public BatBotSmartAI robot = new BatBotSmartAI();

    @Override
    public void init() {
        robot.init(hardwareMap, gamepad1, gamepad2, telemetry);
        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
        */
        //odo.recalibrateIMU();
        //odo.resetPosAndIMU();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.isRed = STEMperFiConstants.ALLIANCE_RED.equals(blackboard.getOrDefault(STEMperFiConstants.BLACKBOARD_KEY_ALLIANCE, STEMperFiConstants.ALLIANCE_RED));
        robot.setAllLedsSolid(robot.isRed ? Color.RED: Color.BLUE);
        robot.limelight.pipelineSwitch(robot.isRed ? STEMperFiConstants.LIMELIGHT_PIPELINE_RED : STEMperFiConstants.LIMELIGHT_PIPELINE_BLUE);
        telemetry.addData("Alliance", blackboard.get(STEMperFiConstants.BLACKBOARD_KEY_ALLIANCE));
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robot.limelight.start();
    //    robot.isRed = blackboard.getOrDefault(STEMperFiConstants.BLACKBOARD_KEY_PATTERN, STEMperFiConstants.PATTERN_21_GPP) == STEMperFiConstants.ALLIANCE_RED;

    }

    @Override
    public void loop() {
        robot.startLoop();

        // Drive
        robot.mecanumDrive();



        // Indexer
        robot.indexer(false);
        // INTAKE
        robot.intake();

        // FLYWHEEL
        robot.flywheel();

        robot.detectGoal(1_000);

        robot.setTurretPower();

        // SHOOTER
        robot.shoot();
    }
}