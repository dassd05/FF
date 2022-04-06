package org.firstinspires.ftc.teamcode.drive.testing;

import static org.firstinspires.ftc.teamcode.drive.Constants.CAP_HIGH;
import static org.firstinspires.ftc.teamcode.drive.Constants.CAP_LOW;
import static org.firstinspires.ftc.teamcode.drive.Constants.FAST_COOL_DOWN;
import static org.firstinspires.ftc.teamcode.drive.Constants.LINKAGE_ADJUSTMENT;
import static org.firstinspires.ftc.teamcode.drive.Constants.NORMAL_COOL_DOWN;
import static org.firstinspires.ftc.teamcode.drive.Constants.SLIDES_ADJUSTMENT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.GamepadSystems.GamepadListenerEx;
import org.firstinspires.ftc.teamcode.drive.Robot;

@TeleOp (name = "cap")
public class Cap extends LinearOpMode {
    public DcMotorEx slides1, slides2;

    public void runOpMode() throws InterruptedException {

        slides1 = hardwareMap.get(DcMotorEx.class, "slides1");
        slides2 = hardwareMap.get(DcMotorEx.class, "slides2");

        slides1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("slides 1 position", slides1.getCurrentPosition());
            telemetry.addData("slides 2 position", slides2.getCurrentPosition());

            telemetry.update();
        }
    }
}
