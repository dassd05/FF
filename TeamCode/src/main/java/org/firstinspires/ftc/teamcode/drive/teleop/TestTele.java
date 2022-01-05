package org.firstinspires.ftc.teamcode.drive.teleop;

@Deprecated
public class TestTele {}

//package org.firstinspires.ftc.teamcode.drive.TeleOp;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.drive.Robot;
//import org.firstinspires.ftc.teamcode.drive.GamepadSystems.GamepadListenerEx;
//
//
//@TeleOp(name = "TestTele", group = "1")
//public class TestTele extends LinearOpMode {
//
//    Robot r = new Robot(); //instantiate Robot object
//
//    boolean runThouPID = true;
//
//    boolean intakeOn = false;
//    boolean carouselOn = false;
//
//    double target = 450;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        r.telemetry = telemetry;
//        r.init(hardwareMap);
//
//        GamepadListenerEx gamepadListener1 = new GamepadListenerEx(gamepad1) {
//            @Override
//            public void onButtonPress(Button button) {
//                super.onButtonPress(button);
//            }
//        };
//        //toggles intake on/off with right bumper
//        GamepadListenerEx gamepadListener2 = new GamepadListenerEx(gamepad2) {
//            @Override
//            public void onButtonPress(Button button) {
//                super.onButtonPress(button);
//                if (button == Button.right_bumper)
//                    intakeOn = !intakeOn;
//                if (button == Button.left_bumper)
//                    carouselOn = !carouselOn;
//            }
//        };
//
//        waitForStart();
//
//        if (opModeIsActive()) {
//
//            r.slidesTimer.reset();
//
//            while (opModeIsActive()) {
//
//                r.clearCache();
//
//                double forward = -gamepad1.left_stick_y;
//                double turn = gamepad1.right_stick_x;
//
//                // left bumper -> slow down drive
//                if (gamepad1.left_bumper)
//                    r.setTankPowers(forward, turn, 0.3);
//                else
//                    r.setTankPowers(forward, turn, 1.0);
//
//
//                // right trigger hold -> reverse carousel direction
//                if (carouselOn) {
//                    if (gamepad2.right_trigger > 0.5) {
//                        r.carousel1.setPower(-1);
//                        r.carousel2.setPower(1);
//                    } else {
//                        r.carousel1.setPower(1);
//                        r.carousel2.setPower(-1);
//                    }
//                } else {
//                    r.carousel1.setPower(0);
//                    r.carousel2.setPower(0);
//                }
//
//
//                // gp2 right bumper -> on/off intake
//                // right trigger hold -> reverse power
//                if (intakeOn)
//                    if (gamepad2.right_trigger > 0.5) //works since you don't have to hold right bumper
//                        r.intakeReverse();
//                    else
//                        r.intakeOn();
//                else
//                    r.intakeOff();
//
//                r.slides1.setPower(.5);
//                r.slides2.setPower(.5);
//
//
////                if (Math.abs(r.getSlides1CurrentPosition() - target) < 20 || Math.abs(r.getSlides2CurrentPosition() - target) < 20) {
////                    r.slides1.setVelocity(0);
////                    r.slides2.setVelocity(0);
////                }
////                else {
////                    r.slides1.setVelocity(400);
////                    r.slides2.setVelocity(400);
////                }
//
//                //r.linearSlidesPID(target, r.slidesTimer.time(), runThouPID);
//
//
//
//                r.updateIntakeState(); //state machine stuff
//
//                telemetry.addData("slides error", r.errorSlides1);
//                //for motor direction debugging
//
//                telemetry.addData("slides1", r.slides1.getCurrentPosition());
//                telemetry.addData("slides2", r.slides2.getCurrentPosition());
//
//                telemetry.update();
//                gamepadListener1.update();
//                gamepadListener2.update();
//            }
//        }
//    }
//}