package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;

@Autonomous(group = "1", name = "Test Turn")
public class testTurn extends LinearOpMode {

    Robot r = new Robot();

    double angle = 50;

    @Override
    public void runOpMode() throws InterruptedException {

        r.telemetry = telemetry;
        r.init(hardwareMap);

        while (!opModeIsActive())
            r.updateAll();

        waitForStart();

        if (isStopRequested()) return;

        r.turnTimer.reset();

        while (opModeIsActive()) {
            r.clearCache();

            r.runPID = (Math.abs(r.getAngle()) < Math.abs(angle));

            r.turnPID(angle, r.turnTimer.time(), r.runPID);

            r.updateAll();

            telemetry.addData("angle", r.getAngle());
            telemetry.addData("p", r.P);
            telemetry.addData("i", r.I);
            telemetry.addData("d", r.D);
            telemetry.addData("time", r.turnTimer.time());
            telemetry.addData("front left", r.frontLeftPosition());
            telemetry.addData("back left", r.backLeftPosition());
            telemetry.addData("front right", r.frontRightPosition());
            telemetry.addData("back right", r.backRightPosition());
            telemetry.update();
        }
    }
}
