package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="Dingo Color", group="Dingo")
public class AutoColor extends LinearOpMode
{
    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    private final BatBot robot = new BatBot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, gamepad1, gamepad2, joinedTelemetry, blackboard);
        robot.odo.recalibrateIMU();
        robot.odo.resetPosAndIMU();
        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeInInit()) {
            robot.startLoop();
            robot.indexer(true);
        }
        runtime.reset();

        while (opModeIsActive()) {
            robot.startLoop();
            robot.indexer(false);
            double currentColor = robot.determineColor();
            robot.indexerContents[0] = currentColor;
            robot.indexerContents[1] = currentColor;
            robot.indexerContents[2] = currentColor;
            robot.setIndexerLeds();
            joinedTelemetry.update();
        }
    }
}
