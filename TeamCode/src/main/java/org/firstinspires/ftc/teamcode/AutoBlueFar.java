package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="Auto Blue Far", group="Meet 3 Far")
public class AutoBlueFar extends AutoRedFar
{
    public AutoBlueFar() {
        super();
        isRed = false;
    }
}
