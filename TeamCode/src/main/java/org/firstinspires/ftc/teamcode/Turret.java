package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "Turret adjust", group = "util")
public class Turret extends OpMode {
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
        robot.turretMotor.set(0);
    }

    @Override
    public void loop() {
        robot.startLoop();
        robot.calibrateTurret();
        telemetry.update();
    }
}