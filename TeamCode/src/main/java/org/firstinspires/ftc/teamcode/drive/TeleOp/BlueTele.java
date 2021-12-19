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
    boolean carouselRampUp = false;

    double carouselPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        r.telemetry = telemetry;
        r.init(hardwareMap);

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


            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // left bumper -> slow down drive
            if (gamepad1.left_bumper)
                r.setTankPowers(forward, turn, 0.3);
            else
                r.setTankPowers(forward, turn, 1.0);


            //TODO: all the deploy state stuff

            // right bumper -> zoom zoom adjustment
            // up and down -> vertical slides adjust
            // right -> horizontal extends out of robot
            // left -> horizontal extends into robot
            if (gamepad1.right_bumper) {
                if (gamepad1.dpad_right)
                    r.linkageAdjust(LINKAGE_ADJUSTMENT * 5);
                if (gamepad1.dpad_left)
                    r.linkageAdjust(-LINKAGE_ADJUSTMENT * 5);
                if (gamepad1.dpad_up)
                    r.slidesAdjust(SLIDES_ADJUSTMENT * 5);
                if (gamepad1.dpad_down)
                    r.slidesAdjust(-SLIDES_ADJUSTMENT * 5);
            } else {
                if (gamepad1.dpad_right)
                    r.linkageAdjust(LINKAGE_ADJUSTMENT);
                if (gamepad1.dpad_left)
                    r.linkageAdjust(-LINKAGE_ADJUSTMENT);
                if (gamepad1.dpad_up)
                    r.slidesAdjust(SLIDES_ADJUSTMENT);
                if (gamepad1.dpad_down)
                    r.slidesAdjust(-SLIDES_ADJUSTMENT);
            }



            // gp2 left bumper -> carousel on
            carouselRampUp = gamepad2.left_bumper;

            if (carouselRampUp)
                carouselPower += .005;
            else
                carouselPower = 0;

            // right trigger hold -> reverse carousel direction
            if (gamepad2.right_trigger > 0.5) {
                r.carousel1.setPower(-carouselPower);
                r.carousel2.setPower(-carouselPower);
            } else {
                r.carousel1.setPower(carouselPower);
                r.carousel2.setPower(carouselPower);
            }


            // gp2 right bumper -> on/off intake
            // right trigger hold -> reverse power
            if (intakeOn)
                if (gamepad2.right_trigger > 0.5) //works since you don't have to hold right bumper
                    r.intakeReverse();
                else
                    r.intakeOn();
            else
                r.intakeOff();



            r.updateAllStates(); //state machine stuff

            telemetry.update();
            gamepadListener1.update();
            gamepadListener2.update();
        }
    }
}