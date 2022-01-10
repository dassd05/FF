package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.GamepadSystems.GamepadListenerEx;

import static org.firstinspires.ftc.teamcode.drive.Constants.*;
import static org.firstinspires.ftc.teamcode.drive.Robot.*;

@TeleOp(name = "TeleOp", group = "1")
public class Teleop extends LinearOpMode {

    Robot r = new Robot(); //instantiate Robot object

    public ElapsedTime buttonCoolDown = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    boolean intakeOn = false;
    boolean carouselOn = false;

    @Override
    public void runOpMode() throws InterruptedException {

        r.telemetry = telemetry;
        r.init(hardwareMap);

        GamepadListenerEx gamepadListener1 = new GamepadListenerEx(gamepad1) {
            @Override
            public void onButtonPress(Button button) {
                super.onButtonPress(button);
                if (button == Button.left_bumper)
                    r.dropoffBox();
            }
        };
        //toggles intake on/off with right bumper
        GamepadListenerEx gamepadListener2 = new GamepadListenerEx(gamepad2) {
            @Override
            public void onButtonPress(Button button) {
                super.onButtonPress(button);
                if (button == Button.right_bumper)
                    intakeOn = !intakeOn;
                if (button == Button.left_bumper)
                    carouselOn = !carouselOn;

                //TODO: adjust to driver preference
                if (button == Button.dpad_up && r.deploymentState == deployState.REST)
                    r.deployTop();
                if (button == Button.dpad_up && r.deploymentState == deployState.MIDDLE)
                    r.deployTop();
                if (button == Button.dpad_up && r.deploymentState == deployState.SHARED)
                    r.deployMiddle();

                if (button == Button.dpad_down && r.deploymentState != deployState.REST)
                    r.deployRest();
                else if (button == Button.dpad_down)
                    r.deployShared();

                if (button == Button.dpad_left || button == Button.dpad_right)
                    r.deployMiddle();

                if (button == Button.x && r.boxState == BoxState.COLLECT)
                    r.liftBox();
                else if (button == Button.x && r.boxState == BoxState.UP)
                    r.collectBox();

            }
        };

        waitForStart();

        buttonCoolDown.reset();

        while (opModeIsActive()) {

            r.clearCache();

            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // right trigger -> slow down drive
            if (gamepad1.right_trigger > .3)
                r.setTankPowers(forward, turn, 0.3);
            else
                r.setTankPowers(forward, turn, 1.0);


            // gp2 left bumper -> carousel on
            if (carouselOn) {
                if (gamepad2.right_trigger > 0.5) {
                    r.carousel1.setPower(-1);
                    r.carousel2.setPower(1);
                } else {
                    r.carousel1.setPower(1);
                    r.carousel2.setPower(-1);
                }
            } else {
                r.carousel1.setPower(0);
                r.carousel2.setPower(0);
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


            // right bumper -> slow down adjustment
            // up and down -> vertical slides adjust
            // right -> horizontal extends out of robot
            // left -> horizontal extends into robot
            if (gamepad1.right_bumper) {
                if (buttonCoolDown.time() >= NORMAL_COOL_DOWN)
                    adjustStuff();
            } else {
                if (buttonCoolDown.time() >= FAST_COOL_DOWN)
                    adjustStuff();
            }

            r.updateAll();

            telemetry.addData("power", r.power);
            telemetry.addData("desired slides position", r.desiredSlidesPosition);
            telemetry.addData("slides 1 position", r.getSlides1CurrentPosition());
            telemetry.addData("slides 2 position", r.getSlides2CurrentPosition());
            telemetry.addData("state", r.deploymentState);

            telemetry.update();
            gamepadListener1.update();
            gamepadListener2.update();
        }
    }

    public void adjustStuff() {
        if (gamepad1.dpad_right)
            r.linkageAdjust(LINKAGE_ADJUSTMENT);
        if (gamepad1.dpad_left)
            r.linkageAdjust(-LINKAGE_ADJUSTMENT);
        if (gamepad1.dpad_up)
            r.slidesAdjust(SLIDES_ADJUSTMENT);
        if (gamepad1.dpad_down)
            r.slidesAdjust(-SLIDES_ADJUSTMENT);

        buttonCoolDown.reset();
    }
}