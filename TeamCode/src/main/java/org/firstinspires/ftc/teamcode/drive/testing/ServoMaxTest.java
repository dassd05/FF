package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "LinkageMaxTest", group = "1")
public class ServoMaxTest extends LinearOpMode {

    public Servo linkage1, linkage2;

    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public double position = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        linkage1 = hardwareMap.get(Servo.class, "linkage1");
        linkage2 = hardwareMap.get(Servo.class, "linkage2");

        waitForStart();

        while (opModeIsActive()) {

            linkage1.setPosition(1.0 - position);
            linkage2.setPosition(position);

            if(timer.time() >= 100 && !gamepad1.a) {
                position += .01;
                timer.reset();
            }


            telemetry.addData("position", position);
            telemetry.update();
        }
    }
}