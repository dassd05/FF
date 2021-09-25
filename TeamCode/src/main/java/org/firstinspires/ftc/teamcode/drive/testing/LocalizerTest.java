package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.Robot;

@TeleOp(name = "TeleOp Blue", group = "1")
public class LocalizerTest extends LinearOpMode {

    Robot r = new Robot(); //instantiate Robot object

    @Override
    public void runOpMode() {

        r.telemetry = telemetry;
        r.dashboard = FtcDashboard.getInstance();
        r.init(hardwareMap); //init hardware
        telemetry = new MultipleTelemetry(telemetry, r.dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {

            double forward = gamepad1.right_stick_y; //might need to reverse joystick direction
            double turn = gamepad1.left_stick_x; //ditto ^

            if(gamepad1.right_bumper) {
                r.setTankPowers((Range.clip(forward + turn, -1, 1)) / 4.0,
                        (Range.clip(forward - turn, -1, 1)) / 4.0);
            } else {
                r.setTankPowers((Range.clip(forward + turn, -1, 1)) * 0.7,
                        (Range.clip(forward - turn, -1, 1)) * 0.7);
            }

            r.updatePos(r.getX(), r.getY(), r.getTheta(), r.odoTimer.time());

            telemetry.addData("x", r.getX());
            telemetry.addData("y", r.getY());
            telemetry.addData("theta", r.getTheta());
            telemetry.addData("imu", r.getAngle());
            telemetry.update();
        }
    }
}