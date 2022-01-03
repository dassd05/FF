package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.gamepad.GamepadListenerEx;
import org.firstinspires.ftc.teamcode.drive.Robot;


public class TeleOp extends LinearOpMode {
    public Robot robot = new Robot(hardwareMap, telemetry);

    public ElapsedTime buttonCoolDown = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public boolean intakeOn = false;
    public boolean carouselOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        robot.dashboardInit();

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
                if (button == Button.left_bumper)
                    carouselOn = !carouselOn;

                //TODO: adjust to driver preference
                if (button == Button.dpad_up && robot.deploymentState == Robot.DeployState.REST)
                    robot.deployTop();
                if (button == Button.dpad_up && robot.deploymentState == Robot.DeployState.MIDDLE)
                    robot.deployTop();
                if (button == Button.dpad_up && robot.deploymentState == Robot.DeployState.SHARED)
                    robot.deployMiddle();

                if (button == Button.dpad_down && robot.deploymentState != Robot.DeployState.REST)
                    robot.deployRest();
                else if (button == Button.dpad_down)
                    robot.deployShared();

                if (button == Button.dpad_left || button == Button.dpad_right)
                    robot.deployMiddle();
            }
        };

        waitForStart();

        buttonCoolDown.reset();

        while (opModeIsActive()) {

            robot.clearCache();

            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // left bumper -> slow down drive
            if (gamepad1.left_bumper)
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

            robot.moveSlides(robot.desiredSlidesPosition, robot.slidesPower);


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


            robot.updateAllStates(); //state machine stuff

            telemetry.update();
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
