package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.GamepadSystems.GamepadListenerEx;

import static org.firstinspires.ftc.teamcode.drive.Constants.Constants.*;
import static org.firstinspires.ftc.teamcode.drive.Robot.*;

@TeleOp(name = "TeleOp", group = "1")
public class BlueTele extends LinearOpMode {

    Robot r = new Robot(); //instantiate Robot object

    boolean intakeOn = false;

    @Override
    public void runOpMode() throws InterruptedException {

        r.telemetry = telemetry;
        r.dashboard = FtcDashboard.getInstance();
        r.init(hardwareMap); //init hardware
        telemetry = new MultipleTelemetry(telemetry, r.dashboard.getTelemetry());

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

            if (gamepad1.left_bumper)
                r.setTankPowers(forward, turn, 0.3);
            else
                r.setTankPowers(forward, turn, 1.0);

            if (gamepad1.right_bumper) {
                if (gamepad1.dpad_right)
                    r.adjust(.01);
                if (gamepad1.dpad_left)
                    r.adjust(-.01);
            } else {
                if (gamepad1.dpad_right)
                    r.adjust(.001);
                if (gamepad1.dpad_left)
                    r.adjust(-.001);
            }

            if (gamepad2.left_bumper) {
                r.carousel1.setPower(1.0);
                r.carousel2.setPower(1.0);
            } else {
                r.carousel1.setPower(0.0);
                r.carousel2.setPower(0.0);
            }

            if (intakeOn)
                r.intake.setPower(INTAKE_POWER);
            else
                r.intake.setPower(0.0);

            r.updateAllStates();

            telemetry.update();
            gamepadListener1.update();
            gamepadListener2.update();
        }
    }
}