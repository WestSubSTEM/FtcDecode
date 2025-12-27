package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auto Red Move", group="Meet 2 Move")
public class AutoRedMove extends LinearOpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    private final BatBot robot = new BatBot();

    public boolean isRed = true;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, gamepad1, gamepad2, telemetry);
        robot.odo.recalibrateIMU();
        robot.odo.resetPosAndIMU();

        while (opModeInInit()) {
            robot.odo.resetPosAndIMU();
            sleep(250);
            telemetry.addData(">", "Robot Heading = %4.0f", robot.odo.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            robot.startLoop();
            robot.indexer(true);
        }
        robot.indexerServoPosition = STEMperFiConstants.INDEX_1;
        robot.indexer(true);
        int time = 750;
        double speed = 0.4;
        if (!isRed) {
            strafeRightTime(speed, time);
        } else {
            strafeLeftTime(speed, time);
        }
        sleep(1_000);
    }


    // **********  HIGH Level driving functions.  ********************
    public void turnTime(double maxTurnSpeed, long timeMs) {
        long start = System.currentTimeMillis();
        robot.mecanum.driveRobotCentric(0, 0, maxTurnSpeed);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
            sleep(100);
            robot.odo.update();
            telemetry.addData("current y: ", robot.odo.getPosY(DistanceUnit.MM));
            telemetry.update();
        }
        robot.mecanum.driveRobotCentric(0, 0, 0);
    }

    public void driveTime(double maxDriveSpeed, long timeMs) {
        long start = System.currentTimeMillis();
        robot.mecanum.driveRobotCentric(0, maxDriveSpeed, 0);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
            sleep(100);
        }
        robot.mecanum.driveRobotCentric(0, 0, 0);
    }


    public void strafeLeftTime(double maxDriveSpeed, long timeMs) {
        long start = System.currentTimeMillis();
        robot.mecanum.driveRobotCentric(-maxDriveSpeed, 0, 0);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
            sleep(100);
            robot.odo.update();
            telemetry.addData("current y: ", robot.odo.getPosY(DistanceUnit.MM));
            telemetry.update();
        }
        robot.mecanum.driveRobotCentric(0, 0, 0);
    }

    public void strafeRightTime(double maxDriveSpeed, long timeMs) {
        long start = System.currentTimeMillis();
        robot.mecanum.driveRobotCentric(maxDriveSpeed, 0, 0);
        // Ensure that the OpMode is still active
        while (opModeIsActive() && ((System.currentTimeMillis() - start) < timeMs)) {
            sleep(100);
            robot.odo.update();
            telemetry.addData("current y: ", robot.odo.getPosY(DistanceUnit.MM));
            telemetry.update();
        }

        robot.mecanum.driveRobotCentric(0, 0, 0);
    }

}
