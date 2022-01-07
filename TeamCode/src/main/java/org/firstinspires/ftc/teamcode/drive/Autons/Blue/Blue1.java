package org.firstinspires.ftc.teamcode.drive.Autons.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Autons.Vision.BoxPositionDetection;
import org.firstinspires.ftc.teamcode.drive.Robot;

import static org.firstinspires.ftc.teamcode.drive.Autons.Vision.BoxPositionDetection.pipeline;
import static org.firstinspires.ftc.teamcode.drive.Robot.*;

@Autonomous(group = "A", name = "Blue Left", preselectTeleOp = "BlueTele")
public class Blue1 extends LinearOpMode {

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

        sleep(150); //just to make sure webcam is properly initialized before updateBoxPosition()

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
                    switch (LeftBlueState) {

                    }
                    break;

                case RIGHT_POSITION:
                    switch (RightBlueState) {

                    }
                    break;

                case MIDDLE_POSITION:
                    switch (MiddleBlueState) {

                    }
                    break;
            }

            r.updateAllStates();
            r.updatePos(r.getX(), r.getY(), r.getTheta(), r.odoTimer.time());

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
            telemetry.addData("Right Position", "Waiting for start");
            WhatPosition = ThisPosition.RIGHT_POSITION;
            telemetry.update();
        } else if (pipeline.position == BoxPositionDetection.BoxPosition.MIDDLE){
            telemetry.addData("Middle Position", "Waiting for start");
            WhatPosition = ThisPosition.MIDDLE_POSITION;
        } else if (pipeline.position == BoxPositionDetection.BoxPosition.LEFT){
            telemetry.addData("Left Position", "Waiting for start");
            WhatPosition = ThisPosition.LEFT_POSITION;
        }
        telemetry.update();
        sleep(75); //so we don't burn cpu cycles
    }
}
