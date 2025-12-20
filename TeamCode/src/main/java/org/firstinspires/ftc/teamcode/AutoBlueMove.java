package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Blue Move", group="Meet 2 Move")
public class AutoBlueMove extends AutoRedMove
{
    public AutoBlueMove() {
        super();
        isRed = false;
    }
}
