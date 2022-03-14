package org.firstinspires.ftc.teamcode.drive.teleop;

import android.util.Log;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.gamepad.GamepadListenerEx;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.util.HtmlFormatter;


@TeleOp(name = "TeleOp", group="1")
public class Teleop extends LinearOpMode {
    public Robot robot;

    public ElapsedTime buttonCoolDown = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    boolean intakeOn = false;
    boolean carouselOn = false;
    boolean lastCarouselOn = false;

    double power = 0.0;
    int cycles = 0;

    public double Some_position = Constants.CAP_HIGH;
    public double Some_adjustment = 0.0;
    boolean capUp = true;

    public int songID = 0;
    public boolean songPreloaded = false;

    public CRServo carouselServo;

    public ElapsedTime rumbleCoolDown = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        robot.dashboardInit();

        carouselServo = hardwareMap.get(CRServo.class, "carousel2");

        GamepadListenerEx gamepadListener1 = new GamepadListenerEx(gamepad1) {
            @Override
            public void onButtonPress(Button button) {
                super.onButtonPress(button);
                if (button == Button.left_bumper && robot.getDeployState() != Robot.DeployState.REST)
                    robot.boxDrop();
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
                if (button == Button.dpad_up && robot.getDeployState() == Robot.DeployState.REST)
                    robot.deployTop();
                if (button == Button.dpad_up && robot.getDeployState() == Robot.DeployState.MIDDLE)
                    robot.deployTop();
                if (button == Button.dpad_up && robot.getDeployState() == Robot.DeployState.SHARED)
                    robot.deployMiddle();

                if (button == Button.dpad_down && robot.getDeployState() != Robot.DeployState.REST)
                    robot.deployRest();
                else if (button == Button.dpad_down) {
                    robot.deployShared();
                    robot.linkageAdjust(-.2);
                }

                if (button == Button.dpad_left || button == Button.dpad_right)
                    robot.deployMiddle();

                if (button == Button.x && robot.getBoxState() == Robot.BoxState.COLLECT)
                    robot.boxUp();
                else if (button == Button.x && robot.getBoxState() == Robot.BoxState.UP)
                    robot.boxCollect();

                if (button == Button.b) {
                    capUp = !capUp;
                    Some_adjustment = 0.0;
                }

            }
        };

        songID = hardwareMap.appContext.getResources().getIdentifier("song", "raw", hardwareMap.appContext.getPackageName());
        if (songID != 0) songPreloaded = SoundPlayer.getInstance().preload(hardwareMap.appContext, songID);

        waitForStart();

        if (songPreloaded) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, songID);
            Log.i("TELEOP", "Song started playing");
        } else {
            Log.i("TELEOP", "Song couldn't be preloaded");
        }
        
        buttonCoolDown.reset();
        
        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // right trigger -> slow down drive
            if (gamepad1.right_trigger > .3)
                robot.setTankPowers(forward, turn, 0.3);
            else
                robot.setTankPowers(forward, turn, 1.0);


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

            robot.carousel.setPower(power);
            carouselServo.setPower(power);


            // gp2 right bumper -> on/off intake
            // right trigger hold -> reverse power
            if (intakeOn)
                if (gamepad2.right_trigger > 0.5) //works since you don't have to hold right bumper
                    robot.intakeReverse();
                else
                    robot.intakeOn();
            else
                robot.intakeOff();


            if (capUp)
                Some_position = Constants.CAP_HIGH;
            else
                Some_position = Constants.CAP_LOW;

            robot.capper.setPosition(Some_position + Some_adjustment);


            // right bumper -> slow down adjustment
            // up and down -> vertical slides adjust
            // right -> horizontal extends out of robot
            // left -> horizontal extends into robot
            if (gamepad1.right_bumper) {
                if (buttonCoolDown.time() >= Constants.NORMAL_COOL_DOWN)
                    adjustStuff();
            } else {
                if (buttonCoolDown.time() >= Constants.FAST_COOL_DOWN)
                    adjustStuff();
            }


            // gamepad rumble
            if (rumbleCoolDown.time() > 100) {
                rumbleCoolDown.reset();
                gamepad1.rumble(forward, turn, 100); // rumbles when robot is moving
                gamepad2.rumble(robot.slidesPower, power, 100); // rumbles when slides or carousel moves
            }

            HtmlFormatter caption = new HtmlFormatter().bold().italic();
            HtmlFormatter description = new HtmlFormatter();
            telemetry.addData(caption.textColor("#f57322").format("linkage position"),
                    description.textColor("#f57322").format(robot.linkagePosition + robot.linkageAdjustment));
            telemetry.addData(caption.textColor("#f58f22").format("desired slides position"),
                    description.textColor("#f58f22").format(robot.slidesPosition));
            telemetry.addData(caption.textColor("#f5d222").format("slides 1 position"),
                    description.textColor("#f5d222").format(robot.getSlides1CurrentPosition()));
            telemetry.addData(caption.textColor("#f5e322").format("slides 2 position"),
                    description.textColor("#f5e322").format(robot.getSlides2CurrentPosition()));
            telemetry.addData(caption.textColor("#22f1f5").format("power"),
                    description.textColor("#22f1f5").format(robot.slidesPower));
            telemetry.addData(caption.textColor("purple").format("state"),
                    description.textColor("purple").format(robot.getDeployState()));
            telemetry.addData(caption.textColor("blue").format("power"),
                    description.textColor("blue").format(power));

            robot.updateAll();
            gamepadListener1.update();
            gamepadListener2.update();
        }
    }

    public void adjustStuff() {
        if (gamepad1.dpad_right) {
            if (robot.getDeployState() == Robot.DeployState.SHARED)
                robot.linkageAdjust(Constants.LINKAGE_ADJUSTMENT/1.6);
            else
                robot.linkageAdjust(Constants.LINKAGE_ADJUSTMENT);
        }
        if (gamepad1.dpad_left) {
            if (robot.linkagePosition + robot.linkageAdjustment >= .5) //for zoom zoom adjustment when its super extended
                robot.linkageAdjust(-Constants.LINKAGE_ADJUSTMENT * 2.75);
            else
                robot.linkageAdjust(-Constants.LINKAGE_ADJUSTMENT);
        }
        if (gamepad1.dpad_up)
            robot.slidesAdjust(Constants.SLIDES_ADJUSTMENT);
        if (gamepad1.dpad_down)
            robot.slidesAdjust(-Constants.SLIDES_ADJUSTMENT);

        if (Math.abs(gamepad2.left_stick_y) > .1) {
            if (gamepad2.right_trigger > .5)
                Some_adjustment += gamepad2.left_stick_y/20.0;
            else
                Some_adjustment += gamepad2.left_stick_y/65.0;
        }

        buttonCoolDown.reset();
    }
}
