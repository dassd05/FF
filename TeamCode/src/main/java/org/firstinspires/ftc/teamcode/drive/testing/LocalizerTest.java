package org.firstinspires.ftc.teamcode.drive.testing;

@Deprecated
public class LocalizerTest {}

//package org.firstinspires.ftc.teamcode.drive.testing;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.drive.Robot;
//
//@TeleOp(name = "Localizer Testing")
//public class LocalizerTest extends LinearOpMode {
//
//    Robot r = new Robot(); //instantiate Robot object
//
////    ElapsedTime imuResetTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
////    public boolean resetAngle = false;
//
//    @Override
//    public void runOpMode() {
//
//        r.telemetry = telemetry;
//        r.dashboard = FtcDashboard.getInstance();
//        r.init(hardwareMap);
//        telemetry = new MultipleTelemetry(telemetry, r.dashboard.getTelemetry());
//
//        waitForStart();
//
//        r.odoTimer.reset();
////        imuResetTimer.reset();
//
//        while (opModeIsActive()) {
//
////            if(imuResetTimer.time() > 500) {
////                resetAngle = true;
////                imuResetTimer.reset();
////            }
//
//            double forward = gamepad1.right_stick_y; //might need to reverse joystick direction
//            double turn = gamepad1.left_stick_x; //ditto ^
//
//            if(gamepad1.right_bumper) {
//                r.setTankPowers((Range.clip(forward + turn, -1, 1)) / 4.0,
//                        (Range.clip(forward - turn, -1, 1)) / 4.0);
//            } else {
//                r.setTankPowers((Range.clip(forward + turn, -1, 1)) * 0.7,
//                        (Range.clip(forward - turn, -1, 1)) * 0.7);
//            }
//
//            r.updatePos(r.getX(), r.getY(), r.getTheta(), r.odoTimer.time());
//
////            r.updatePos(r.getX(), r.getY(), r.getTheta(), r.odoTimer.time(), resetAngle);
////            resetAngle = false;
//
//            /**
//             * commented out code shows an application of the imuUpdate parameter in Robot() to reset
//             * odo angle with imu every 500 ms to limit lag but also avoid significant drift
//             *
//             * but... tbh, our imu drift is pretty significant, so unless we use an external gyro
//             * or get a new hub/fix calibration, this method is kinda not practical
//             */
//
//            telemetry.addData("x", r.getX());
//            telemetry.addData("y", r.getY());
//            telemetry.addData("theta", r.getTheta());
//            telemetry.addData("imu", r.getAngle());
//            telemetry.update();
//        }
//    }
//}