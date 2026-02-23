package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "Test Joined Telemetry", group = "Dev")
public class TestJoinedTelemetry extends OpMode {
    public static int COUNTER = 0;
    public static volatile int CHANGE_ME = 0;
    public static int CHANGE_ME_NOT_VOLATILE = 0;
    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(this.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    @Override
    public void init() {
        joinedTelemetry.addLine("init");
        joinedTelemetry.update();
    }

    @Override
    public void loop() {
        joinedTelemetry.addData("Counter", COUNTER);
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