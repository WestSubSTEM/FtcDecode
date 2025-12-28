package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Configurable
@TeleOp(name = "Turret Limelight", group = "util")
public class TurretLimelight extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    public BatBot robot = new BatBot();

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
        robot.odo.resetPosAndIMU();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robot.limelight.pipelineSwitch(STEMperFiConstants.LIMELIGHT_PIPELINE_AUTO);
        robot.limelight.start();
    }

    @Override
    public void loop() {
        robot.startLoop();
        LLResult result = robot.limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            LLResultTypes.FiducialResult bob = fiducialResults.get(0);
            if (bob != null) {
                double xdif = bob.getTargetXDegrees();
                telemetry.addData("Fiducial", "ID: %d, X: %.2f", bob.getFiducialId(), bob.getTargetXDegrees());
                robot.adjustTurret(xdif);
            }
            telemetry.addData("turret", robot.turretPosition);
            telemetry.update();
        }
    }
}