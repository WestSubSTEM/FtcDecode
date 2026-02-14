package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;


@Disabled
@Autonomous(name="turret auto", group="Util")
public class AutoTurret extends LinearOpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    private final BatBot robot = new BatBot();



    @Override
    public void runOpMode() {
        robot.init(hardwareMap, gamepad1, gamepad2, telemetry);
        robot.odo.recalibrateIMU();
        robot.odo.resetPosAndIMU();
        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeInInit()) {
            robot.startLoop();
            telemetry.addData("init", 0);
            sleep(100);
            telemetry.update();
        }
        runtime.reset();

        while (opModeIsActive()) {
            robot.startLoop();
            robot.turretMotor.setPower(gamepad2.right_stick_y);
            telemetry.addData("power:", robot.turretMotor.getPower());
            telemetry.addData("pos: ", -robot.turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
