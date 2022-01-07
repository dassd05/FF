package org.firstinspires.ftc.teamcode.drive.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Deprecated
public class BadAutonRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while(opModeIsActive()) {
            sleep(5000);
        }
    }
}
