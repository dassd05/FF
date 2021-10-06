package org.firstinspires.ftc.teamcode.drive.Autons.Red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Autons.Vision.BoxPositionDetection;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.drive.Autons.Vision.BoxPositionDetection.pipeline;
import static org.firstinspires.ftc.teamcode.drive.Robot.*;

@Autonomous(group = "A", name = "Red Left", preselectTeleOp = "RedTele")
public class Red1 extends LinearOpMode {

    Robot r = new Robot();

    public enum ThisPosition {
        LEFT_POSITION,
        MIDDLE_POSITION,
        RIGHT_POSITION
    }

    public volatile ThisPosition WhatPosition;

    boolean firstTime = true;
    boolean runFSM = false;

    @Override
    public void runOpMode() throws InterruptedException {

        r.telemetry = telemetry;
        r.dashboard = FtcDashboard.getInstance();
        r.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, r.dashboard.getTelemetry());

        r.webcamInit(hardwareMap);

        xPos = 0.0;
        yPos = 0.0;
        thetaPos = 0.0; //ofc might want to change

        sleep(150);

        while (!opModeIsActive())
            updateBoxPosition();

        waitForStart();

        if (isStopRequested()) return;

        r.autonWaitTimer.reset();
        r.odoTimer.reset();
        r.webcam.stopStreaming();

        while (opModeIsActive()) {

            switch (WhatPosition) {
                case LEFT_POSITION:
                    switch (LeftRedState) {

                    }
                    break;

                case RIGHT_POSITION:
                    switch (RightRedState) {

                    }
                    break;

                case MIDDLE_POSITION:
                    switch (MiddleRedState) {

                    }
                    break;
            }

            r.updateAllStates();

            telemetry.addData("x", r.getX());
            telemetry.addData("y", r.getY());
            telemetry.addData("heading", r.getTheta());
            telemetry.update();
        }
    }

    public void updateBoxPosition() {
        if (pipeline.position == null) {
            telemetry.addData("still working on it", "gimme a sec");
        } else if (pipeline.position == BoxPositionDetection.BoxPosition.RIGHT){
            telemetry.addData("Four Rings", "Waiting for start");
            WhatPosition = ThisPosition.RIGHT_POSITION;
            telemetry.update();
        } else if (pipeline.position == BoxPositionDetection.BoxPosition.MIDDLE){
            telemetry.addData("One Ring", "Waiting for start");
            WhatPosition = ThisPosition.MIDDLE_POSITION;
        } else if (pipeline.position == BoxPositionDetection.BoxPosition.LEFT){
            telemetry.addData("Zero Rings", "Waiting for start");
            WhatPosition = ThisPosition.LEFT_POSITION;
        }
        telemetry.update();
        sleep(75);
    }
}
