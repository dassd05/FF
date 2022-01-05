package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.gamepad.GamepadListenerEx;
import org.firstinspires.ftc.teamcode.drive.Robot;


@TeleOp(name = "TeleOp", group="1")
public class Teleop extends LinearOpMode {
    public Robot robot;

    public ElapsedTime buttonCoolDown = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public boolean intakeOn = false;
    public boolean carouselOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        robot.dashboardInit();

        GamepadListenerEx gamepadListener1 = new GamepadListenerEx(gamepad1) {
            @Override
            public void onButtonPress(Button button) {
                super.onButtonPress(button);
                if (button == Button.left_bumper)
                    robot.dropoffBox();
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
//                if (button == Button.dpad_up && robot.deploymentState == Robot.DeployState.REST)
//                    robot.deployTop();
//                if (button == Button.dpad_up && robot.deploymentState == Robot.DeployState.MIDDLE)
//                    robot.deployTop();
//                if (button == Button.dpad_up && robot.deploymentState == Robot.DeployState.SHARED)
//                    robot.deployMiddle();
//
//                if (button == Button.dpad_down && robot.deploymentState != Robot.DeployState.REST)
//                    robot.deployRest();
//                else if (button == Button.dpad_down)
//                    robot.deployShared();
//
//                if (button == Button.dpad_left || button == Button.dpad_right)
//                    robot.deployMiddle();


                if (button == Button.dpad_up) robot.deployTop();
                else if (button == Button.dpad_right) robot.deployMiddle();
                else if (button == Button.dpad_left) robot.deployShared();
                else if (button == Button.dpad_down) robot.deployRest();
            }
        };

        waitForStart();

        buttonCoolDown.reset();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // right trigger -> slow down drive
            if (gamepad1.right_trigger > .3)
                robot.setTankPowers(forward, turn, 0.3);
            else
                robot.setTankPowers(forward, turn, 1.0);

            // right bumper -> zoom zoom adjustment
            // up and down -> vertical slides adjust
            // right -> horizontal extends out of robot
            // left -> horizontal extends into robot
            if (gamepad1.right_bumper) {
                if (buttonCoolDown.time() >= Constants.FAST_COOL_DOWN)
                    adjustStuff();
            } else {
                if (buttonCoolDown.time() >= Constants.NORMAL_COOL_DOWN)
                    adjustStuff();
            }

//            robot.moveSlides(robot.desiredSlidesPosition, robot.slidesPower);
//            robot.moveLinkage(Range.clip(robot.linkagePosition + robot.linkageAdjustment, 0, .9));


            // gp2 left bumper -> carousel on
            if (carouselOn) {
                if (gamepad2.right_trigger > 0.5) {
                    robot.carousel1.setPower(-1);
                    robot.carousel2.setPower(1);
                } else {
                    robot.carousel1.setPower(1);
                    robot.carousel2.setPower(-1);
                }
            } else {
                robot.carousel1.setPower(0);
                robot.carousel2.setPower(0);
            }


            // gp2 right bumper -> on/off intake
            // right trigger hold -> reverse power
            if (intakeOn)
                if (gamepad2.right_trigger > 0.5) //works since you don't have to hold right bumper
                    robot.intakeReverse();
                else
                    robot.intakeOn();
            else
                robot.intakeOff();


            telemetry.addData("power", robot.slidesPower);
            telemetry.addData("desired slides position", robot.slidesPosition);
            telemetry.addData("slides 1 position", robot.getSlides1CurrentPosition());
            telemetry.addData("slides 2 position", robot.getSlides2CurrentPosition());
            telemetry.addData("state", robot.getDeployState());

            robot.updateAll();
            gamepadListener1.update();
            gamepadListener2.update();
        }
    }

    public void adjustStuff() {
        if (gamepad1.dpad_right)
            robot.linkageAdjust(Constants.LINKAGE_ADJUSTMENT);
        if (gamepad1.dpad_left)
            robot.linkageAdjust(-Constants.LINKAGE_ADJUSTMENT);
        if (gamepad1.dpad_up)
            robot.slidesAdjust(Constants.SLIDES_ADJUSTMENT);
        if (gamepad1.dpad_down)
            robot.slidesAdjust(-Constants.SLIDES_ADJUSTMENT);

        buttonCoolDown.reset();
    }
}
