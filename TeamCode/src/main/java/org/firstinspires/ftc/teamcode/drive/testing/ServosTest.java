package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group = "testing")
public class ServosTest extends LinearOpMode {

    //replace servo names as needed

    public Servo boxServo, linkage1, linkage2;
    public CRServo carousel1, carousel2;

    public static double boxServoVal = -1;
    public static double linkage1Val = -1;
    public static double linkage2Val = -1;
    public static double carousel1Val = 0;
    public static double carousel2Val = 0;

    public FtcDashboard dashboard;


    @Override
    public void runOpMode() throws InterruptedException {

        boxServo = hardwareMap.get(Servo.class, "boxServo");
        linkage1 = hardwareMap.get(Servo.class, "linkage1");
        linkage2 = hardwareMap.get(Servo.class, "linkage2");
        carousel1 = hardwareMap.get(CRServo.class, "carousel1");
        carousel2 = hardwareMap.get(CRServo.class, "carousel2");

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            if (boxServoVal != -1) boxServo.setPosition(boxServoVal);
            if (linkage1Val != -1) linkage1.setPosition(linkage1Val);
            if (linkage2Val != -1) linkage2.setPosition(linkage2Val);
            carousel1.setPower(carousel1Val);
            carousel2.setPower(carousel2Val);

            telemetry.update();
        }
    }
}
