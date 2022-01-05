package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.gamepad.GamepadListenerEx;


@TeleOp(name = "Test", group = "1")
public class Test extends LinearOpMode {

    Robot robot;

    boolean runThouPID = true;

    boolean intakeOn = false;
    boolean carouselOn = false;

    double target = 450;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();

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
            }
        };

        waitForStart();

        if (opModeIsActive()) {

            robot.slidesTimer.reset();

            while (opModeIsActive()) {

                robot.clearCache();

                double forward = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;

                // left bumper -> slow down drive
                if (gamepad1.left_bumper)
                    robot.setTankPowers(forward, turn, 0.3);
                else
                    robot.setTankPowers(forward, turn, 1.0);


                // right trigger hold -> reverse carousel direction
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

                robot.slides1.setPower(.5);
                robot.slides2.setPower(.5);


//                if (Math.abs(robot.getSlides1CurrentPosition() - target) < 20 || Math.abs(robot.getSlides2CurrentPosition() - target) < 20) {
//                    robot.slides1.setVelocity(0);
//                    robot.slides2.setVelocity(0);
//                }
//                else {
//                    robot.slides1.setVelocity(400);
//                    robot.slides2.setVelocity(400);
//                }

                //robot.linearSlidesPID(target, robot.slidesTimer.time(), runThouPID);



                robot.updateIntakeState(); //state machine stuff

                telemetry.addData("slides error", robot.errorSlides1);
                //for motor direction debugging

                telemetry.addData("slides1", robot.slides1.getCurrentPosition());
                telemetry.addData("slides2", robot.slides2.getCurrentPosition());

                telemetry.update();
                gamepadListener1.update();
                gamepadListener2.update();
            }
        }
    }
}