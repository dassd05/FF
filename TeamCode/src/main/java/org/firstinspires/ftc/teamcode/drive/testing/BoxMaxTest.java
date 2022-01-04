package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BoxMaxTest", group = "1")
public class BoxMaxTest extends LinearOpMode {

    public Servo boxServo;

    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public double position = .55;

    @Override
    public void runOpMode() throws InterruptedException {

        boxServo = hardwareMap.get(Servo.class, "boxServo");

        waitForStart();

        while (opModeIsActive()) {

            boxServo.setPosition(position);
//
//            if(timer.time() >= 500 && !gamepad1.a) {
//                position += .01;
//                timer.reset();
//            }


            telemetry.addData("position", position);
            telemetry.update();
        }
    }
}