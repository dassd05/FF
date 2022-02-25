package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.GamepadSystems.GamepadListenerEx;

import static org.firstinspires.ftc.teamcode.drive.Constants.*;
import static org.firstinspires.ftc.teamcode.drive.Robot.*;

@TeleOp(name = "Teleop", group = "0")
public class Teleop extends LinearOpMode {

    Robot r = new Robot(); //instantiate Robot object

    public ElapsedTime buttonCoolDown = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    boolean intakeOn = false;
    boolean carouselOn = false;
    boolean lastCarouselOn = false;

    double power = 0.0;
    int cycles = 0;

    public double Some_position = CAP_HIGH;
    public double Some_adjustment = 0.0;
    boolean capUp = true;

    public CRServo carouselServo;

    public ElapsedTime rumbleCooldown = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {

        r.telemetry = telemetry;
        r.init(hardwareMap);

        carouselServo = hardwareMap.get(CRServo.class, "carousel2");

        GamepadListenerEx gamepadListener1 = new GamepadListenerEx(gamepad1) {
            @Override
            public void onButtonPress(Button button) {
                super.onButtonPress(button);
                if (button == Button.left_bumper && r.deploymentState != deployState.REST)
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
                else if (button == Button.dpad_down) {
                    r.deployShared();
                    r.linkageAdjust(-.2);
                }

                if (button == Button.dpad_left || button == Button.dpad_right)
                    r.deployMiddle();

                if (button == Button.x && r.boxState == BoxState.COLLECT)
                    r.liftBox();
                else if (button == Button.x && r.boxState == BoxState.UP)
                    r.collectBox();

                if (button == Button.b) {
                    capUp = !capUp;
                    Some_adjustment = 0.0;
                }

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
                if (lastCarouselOn)
                    cycles += 1;
                if (gamepad2.right_trigger > 0.5)
                    power = .4 * (Math.pow((1+.0135), cycles));
                else
                    power = -.4 * (Math.pow((1+.0135), cycles));

            } else {
                power = 0.0;
                cycles = 0;
            }
            lastCarouselOn = carouselOn;

            r.carousel.setPower(power);
            carouselServo.setPower(power);


            // gp2 right bumper -> on/off intake
            // right trigger hold -> reverse power
            if (intakeOn)
                if (gamepad2.right_trigger > 0.5) //works since you don't have to hold right bumper
                    r.intakeReverse();
                else
                    r.intakeOn();
            else
                r.intakeOff();


            if (capUp)
                Some_position = CAP_HIGH;
            else
                Some_position = CAP_LOW;

            r.capper.setPosition(Some_position + Some_adjustment);


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


            // gamepad rumble
            if (rumbleCooldown.time() > 100) {
                rumbleCooldown.reset();
                gamepad1.rumble(forward, turn, 100); // rumbles when robot is moving
                gamepad2.rumble(r.power, power, 100); // rumbles when slides or carousel moves
            }

            r.updateAll();

            telemetry.addData("linkage position", r.position + r.linkageAdjustment);
            telemetry.addData("desired slides position", r.desiredSlidesPosition);
            telemetry.addData("slides 1 position", r.getSlides1CurrentPosition());
            telemetry.addData("slides 2 position", r.getSlides2CurrentPosition());
            telemetry.addData("power", r.power);
            telemetry.addData("state", r.deploymentState);
            telemetry.addData("power", power);

            telemetry.update();
            gamepadListener1.update();
            gamepadListener2.update();
        }
    }

    public void adjustStuff() {
        if (gamepad1.dpad_right) {
            if (r.deploymentState == deployState.SHARED)
                r.linkageAdjust(LINKAGE_ADJUSTMENT/1.6);
            else
                r.linkageAdjust(LINKAGE_ADJUSTMENT);
        }
        if (gamepad1.dpad_left) {
            if (r.position + r.linkageAdjustment >= .5) //for zoom zoom adjustment when its super extended
                r.linkageAdjust(-LINKAGE_ADJUSTMENT * 2.75);
            else
                r.linkageAdjust(-LINKAGE_ADJUSTMENT);
        }
        if (gamepad1.dpad_up)
            r.slidesAdjust(SLIDES_ADJUSTMENT);
        if (gamepad1.dpad_down)
            r.slidesAdjust(-SLIDES_ADJUSTMENT);

        if (Math.abs(gamepad2.left_stick_y) > .1) {
            if (gamepad2.right_trigger > .5)
                Some_adjustment += gamepad2.left_stick_y/20.0;
            else
                Some_adjustment += gamepad2.left_stick_y/65.0;
        }

        buttonCoolDown.reset();
    }
}