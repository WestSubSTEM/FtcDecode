package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Blue", group="Meet 2")
public class AutoBlueMove extends AutoRedMove
{
    public AutoBlueMove() {
        super();
        isRed = false;
    }
}
