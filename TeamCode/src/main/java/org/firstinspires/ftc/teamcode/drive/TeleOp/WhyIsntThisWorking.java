package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.GamepadSystems.GamepadListenerEx;


@TeleOp(name = "WhyIsntThisWorking", group = "1")
public class WhyIsntThisWorking extends LinearOpMode {

    public DcMotor slides1, slides2;
    public DcMotor intake;

    @Override
    public void runOpMode() throws InterruptedException {

        slides1 = hardwareMap.get(DcMotor.class, "slides1");
        slides2 = hardwareMap.get(DcMotor.class, "slides2");

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                slides1.setPower(.35);
                slides2.setPower(.35);

                telemetry.update();
            }
        }
    }
}