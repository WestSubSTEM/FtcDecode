package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
@Disabled
@Configurable
@TeleOp(name = "Test Joined Telemetry", group = "Dev")
public class TestJoinedTelemetry extends OpMode {
    public static int COUNTER = 0;
    public static volatile int CHANGE_ME = 0;
    public static int CHANGE_ME_NOT_VOLATILE = 0;
    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(this.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    Servo pinkLed, blueLed, yellowLed;

    DigitalChannel laserLeft, laserRight;


    @Override
    public void init() {
        blueLed =  hardwareMap.get(Servo.class, "blueLed");
        pinkLed =  hardwareMap.get(Servo.class, "pinkLed");
        yellowLed =  hardwareMap.get(Servo.class, "yellowLed");
        // Get the digital sensor from the hardware map
        laserRight = hardwareMap.get(DigitalChannel.class, "laser_right");
        // Set the channel as an input
        laserRight.setMode(DigitalChannel.Mode.INPUT);
        laserLeft = hardwareMap.get(DigitalChannel.class, "laser_left");
        // Set the channel as an input
        laserLeft.setMode(DigitalChannel.Mode.INPUT);
        blueLed.setPosition(STEMperFiConstants.GB_LED_PURPLE);
        pinkLed.setPosition(STEMperFiConstants.GB_LED_GREEN);
        yellowLed.setPosition(STEMperFiConstants.GB_LED_GREEN);
        joinedTelemetry.addLine("init");
        joinedTelemetry.update();
    }

    @Override
    public void loop() {
        joinedTelemetry.addData("Counter", COUNTER);
        joinedTelemetry.addData("lr", laserRight.getState());
        joinedTelemetry.addData("lf", laserLeft.getState());
        TestJoinedTelemetry.COUNTER++;
        int x = 3;
        for (int i = 0; i < 10_000; i++) {
            x = x ^ 2;
        }
        joinedTelemetry.addData("Change Me", TestJoinedTelemetry.CHANGE_ME);
        joinedTelemetry.addData("Change No", TestJoinedTelemetry.CHANGE_ME_NOT_VOLATILE);
        joinedTelemetry.update();
    }

//    override fun init() {}
//
//    override fun loop() {
//        joinedTelemetry.addData("Key", "Value")
//        joinedTelemetry.addData("Key2", 50)
//
//        joinedTelemetry.update()
//    }
}