package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp
public class CyclesEfficiencyTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        int cycles = 0;

        waitForStart();
        timer.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                cycles++;
                telemetry.addData("cycles per millisecond", cycles / timer.time());
                telemetry.update();
            }
        }
    }
}
