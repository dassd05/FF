package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.GamepadSystems.GamepadListenerEx;
import org.firstinspires.ftc.teamcode.util.HtmlFormatter;

import static org.firstinspires.ftc.teamcode.drive.Constants.*;
import static org.firstinspires.ftc.teamcode.drive.Robot.*;

@TeleOp(name = "Teleop Red", group = "0")
public class TeleopRed extends LinearOpMode {

    Robot r = new Robot(); //instantiate Robot object

    public ElapsedTime buttonCoolDown = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    boolean intakeOn = false;
    boolean carouselOn = false;
    boolean lastCarouselOn = false;

    double power = 0.0;
    int cycles = 0;

    public static double gain = 3;
    public static double distance_detection = 60.0;

    boolean last_detection = false;

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
                if (button == Button.left_bumper && r.deploymentState != deployState.REST) {
                    if (r.deploymentState == deployState.SHARED_LEFT || r.deploymentState == deployState.SHARED_RIGHT)
                        r.deployShared();
                    else
                        r.deployAlliance();
                }
                if (button == Button.a && r.deploymentState == deployState.CAP_HOVER)
                    r.cap();
                if (button == Button.a && r.deploymentState == deployState.REST)
                    r.hover();
            }
        };
        //toggles intake on/off with right bumper
        GamepadListenerEx gamepadListener2 = new GamepadListenerEx(gamepad2) {
            @Override
            public void onButtonPress(Button button) {
                super.onButtonPress(button);
                if (button == Button.right_bumper)
                    r.intakeOn = !r.intakeOn;
                if (button == Button.left_bumper)
                    carouselOn = !carouselOn;

                r.reverse = gamepad2.right_trigger > .5;

                //TODO: adjust to driver preference
                if (button == Button.dpad_up)
                    r.deployTop();

                if (button == Button.dpad_down && r.deploymentState != deployState.REST)
                    r.deployRest();
                else if (button == Button.dpad_down) {
                    r.deployBottom();
                }

                if (gamepad2.right_trigger > .5) {
                    if (button == Button.dpad_right || button == Button.dpad_left)
                        r.deploySharedRight();
                } else if (gamepad2.left_trigger > .5) {
                    if (button == Button.dpad_left)
                        r.deploySharedLeft();
                    if (button == Button.dpad_right)
                        r.deploySharedRight();
                } else {
                    if (button == Button.dpad_right || button == Button.dpad_left)
                        r.deployMiddle();
                }

            }
        };

        waitForStart();

        buttonCoolDown.reset();

        while (opModeIsActive()) {

            r.clearCache();

            r.boxSensor.setGain((float)gain);
            NormalizedRGBA colors = r.boxSensor.getNormalizedColors();

            double distance = r.boxSensor.getDistance(DistanceUnit.MM);

            if (distance < distance_detection && !last_detection) {
                rumbleCooldown.reset();
            }
            if (distance > distance_detection && last_detection) {
                rumbleCooldown.reset();
            }

            if (distance < distance_detection && rumbleCooldown.time() > 100)
                r.freightLoaded = true;
            else if (distance > distance_detection && last_detection && rumbleCooldown.time() < 500) {
                r.freightLoaded = true;
            } else {
                r.freightLoaded = false;
            }

            last_detection = distance < distance_detection;

            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // right trigger -> slow down drive
            if (gamepad1.right_trigger > .3)
                r.setTankPowers(forward, turn, 0.3);
            else if (r.deploymentState != deployState.REST && r.deploymentState != deployState.SHARED_LEFT && r.deploymentState != deployState.SHARED_RIGHT)
                r.setTankPowers(forward, turn, .6);
            else
                r.setTankPowers(forward, turn, 1.0);


            // gp2 left bumper -> carousel on
            if (carouselOn) {
                if (lastCarouselOn)
                    cycles += 1;
                if (gamepad2.right_trigger > 0.5)
                    power = .5 * (Math.pow((1+.025), cycles));
                else
                    power = -.5 * (Math.pow((1+.025), cycles));

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

            HtmlFormatter caption = new HtmlFormatter().bold().italic();
            HtmlFormatter description = new HtmlFormatter();
            telemetry.addData("freight loaded", r.freightLoaded);
            telemetry.addData("turret", r.turret.getPosition());
            telemetry.addData(caption.textColor("#f57322").format("linkage position"),
                    description.textColor("#f57322").format(r.position + r.linkageAdjustment));
            telemetry.addData(caption.textColor("#f58f22").format("desired slides position"),
                    description.textColor("#f58f22").format(r.desiredSlidesPosition));
            telemetry.addData(caption.textColor("#f5d222").format("slides 1 position"),
                    description.textColor("#f5d222").format(r.getSlides1CurrentPosition()));
            telemetry.addData(caption.textColor("#f5e322").format("slides 2 position"),
                    description.textColor("#f5e322").format(r.getSlides2CurrentPosition()));
            telemetry.addData(caption.textColor("#22f1f5").format("power"),
                    description.textColor("#22f1f5").format(r.power));
            telemetry.addData(caption.textColor("purple").format("state"),
                    description.textColor("purple").format(r.deploymentState));
            telemetry.addData(caption.textColor("blue").format("carousel"),
                    description.textColor("blue").format(power));

            telemetry.update();
            gamepadListener1.update();
            gamepadListener2.update();
        }
    }

    public void adjustStuff() {
        if (gamepad1.dpad_up)
            r.slidesAdjust(SLIDES_ADJUSTMENT);
        if (gamepad1.dpad_down)
            r.slidesAdjust(-SLIDES_ADJUSTMENT);


        if (gamepad1.dpad_right) {
            r.linkageAdjust(LINKAGE_ADJUSTMENT / 1.25);
        }

        if (gamepad1.dpad_left) {
            if (r.position + r.linkageAdjustment >= .5) //for zoom zoom adjustment when its super extended
                r.linkageAdjust(-LINKAGE_ADJUSTMENT * 2.75);
            else
                r.linkageAdjust(-LINKAGE_ADJUSTMENT);
        }

        if (r.linkage1.getPosition() <= Range.scale((LINKAGE_SHARED - .1), 0, 1, LINKAGE_1_MAX_IN, LINKAGE_1_MAX_OUT)) {
            if (r.deploymentState != deployState.REST) {
                if (-.016 * gamepad2.left_stick_x < 0 && r.turret.getPosition() > 0) {
                    r.turretAdjust(-.016 * gamepad2.left_stick_x);
                } else if (-.016 * gamepad2.left_stick_x > 0 && r.turret.getPosition() < 1) {
                    r.turretAdjust(-.016 * gamepad2.left_stick_x);
                }
            }
        }
        r.armAdjust(.008 * gamepad2.right_stick_y);

        buttonCoolDown.reset();
    }
}