package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.GamepadSystems.GamepadListenerEx;

import static org.firstinspires.ftc.teamcode.drive.Constants.Constants.*;
import static org.firstinspires.ftc.teamcode.drive.Robot.*;

@TeleOp(name = "Test TeleOp", group = "1")
public class TestTele extends LinearOpMode {

    boolean intakeOn = false;
    boolean carouselRampUp = false;

    double carouselPower = 0.0;

    public DcMotor Left1, Left2, Right1, Right2;

    public DcMotor intake;

    public CRServoImplEx carousel1;

    @Override
    public void runOpMode() throws InterruptedException {

        Left1 = hardwareMap.get(DcMotor.class, "frontLeft");
        Left2 = hardwareMap.get(DcMotor.class, "backLeft");
        Right1 = hardwareMap.get(DcMotor.class, "frontRight");
        Right2 = hardwareMap.get(DcMotor.class, "backRight");

        Left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Right1.setDirection(DcMotorSimple.Direction.REVERSE);
        Left1.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");

        carousel1 = hardwareMap.get(CRServoImplEx.class, "carousel1");

        GamepadListenerEx gamepadListener1 = new GamepadListenerEx(gamepad1) {
            @Override
            public void onButtonPress(Button button) {
                super.onButtonPress(button);
            }
        };
        //toggles intake on/off with right bumper
        GamepadListenerEx gamepadListener2 = new GamepadListenerEx(gamepad2) {
            @Override
            public void onButtonPress(Button button) {
                super.onButtonPress(button);
                if (button == Button.right_bumper)
                    intakeOn = !intakeOn;
            }
        };

        waitForStart();

        while (opModeIsActive()) {

            double forward = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // left bumper -> slow down drive
            if (gamepad1.left_bumper)
                setTankPowers(forward, turn, 0.3);
            else
                setTankPowers(forward, turn, 1.0);


            // gp2 right bumper -> on/off intake
            // right trigger hold -> reverse power
            if (intakeOn)
                if (gamepad2.right_trigger > 0.5) //works since you don't have to hold right bumper
                    intake.setPower(-INTAKE_POWER);
                else
                    intake.setPower(INTAKE_POWER);
            else
                intake.setPower(0.0);


            carouselRampUp = gamepad2.left_bumper;

            if (carouselRampUp)
                carouselPower += .005;
            else
                carouselPower = 0;

            // right trigger hold -> reverse carousel direction
            if (gamepad2.right_trigger > 0.5)
                carousel1.setPower(-carouselPower);
             else
                carousel1.setPower(carouselPower);


            telemetry.update();
            gamepadListener1.update();
            gamepadListener2.update();
        }
    }

    public void setTankPowers(double forward, double turn, double multiplier) {
        Left1.setPower((forward + turn) * multiplier);
        Left2.setPower((forward + turn) * multiplier);
        Right1.setPower((forward - turn) * multiplier);
        Right2.setPower((forward - turn) * multiplier);
    }

}