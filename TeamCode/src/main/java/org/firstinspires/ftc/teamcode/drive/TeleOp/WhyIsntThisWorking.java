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

    public DcMotorEx slides1, slides2;

    @Override
    public void runOpMode() throws InterruptedException {

        slides1 = hardwareMap.get(DcMotorEx.class, "slides1");
        slides2 = hardwareMap.get(DcMotorEx.class, "slides2");

        slides1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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