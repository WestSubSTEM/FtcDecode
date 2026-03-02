package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pushbot", group = "Meet3")
public class Pushbot extends OpMode {
    public MecanumDrive mecanum;
    public GamepadEx gp1;
    Motor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    @Override
    public void init() {
        frontLeftMotor = new Motor(hardwareMap, "fl drive");
        frontRightMotor = new Motor(hardwareMap, "fr drive");
        backLeftMotor = new Motor(hardwareMap, "bl drive");
        backRightMotor = new Motor(hardwareMap, "br drive");
        gp1 = new GamepadEx(gamepad1);
        mecanum = new MecanumDrive(backRightMotor, backLeftMotor, frontRightMotor, frontLeftMotor);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
        */
        //odo.recalibrateIMU();Teleop
        //odo.resetPosAndIMU();
    }

    public void mecanumDrive() {
        double lx = gp1.getLeftX();
        double ly = gp1.getLeftY();
        double rx = gp1.getRightX();
        if (gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            lx = lx / 2;
            ly = ly / 2;
            rx = rx / 2;
        }

        mecanum.driveRobotCentric(
                lx,
                ly,
                rx
        );
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    @Override
    public void loop() {
        mecanumDrive();

        if (gamepad2.dpad_up) {
            frontLeftMotor.set(.3);
        }
        if (gamepad2.dpad_down) {
            backLeftMotor.set(.3);
        }
        if (gamepad2.dpad_right) {
            frontRightMotor.set(.3);
        }
        if (gamepad2.dpad_left) {
            backRightMotor.set(.3);
        }
    }
}