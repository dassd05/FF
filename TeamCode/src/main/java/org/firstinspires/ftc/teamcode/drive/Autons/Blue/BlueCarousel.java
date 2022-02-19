package org.firstinspires.ftc.teamcode.drive.Autons.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Autons.Vision.BoxPositionDetection;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.drive.Autons.Vision.BoxPositionDetection.*;
import static org.firstinspires.ftc.teamcode.drive.Autons.Vision.BoxPositionDetection.BoxDetection.avg1;
import static org.firstinspires.ftc.teamcode.drive.Autons.Vision.BoxPositionDetection.BoxDetection.avg2;
import static org.firstinspires.ftc.teamcode.drive.Autons.Vision.BoxPositionDetection.BoxDetection.avg3;
import static org.firstinspires.ftc.teamcode.drive.Constants.CAP_HIGH;
import static org.firstinspires.ftc.teamcode.drive.Robot.*;

@Autonomous(group = "A", name = "Blue Carousel", preselectTeleOp = "Teleop")
public class BlueCarousel extends LinearOpMode {

    Robot r = new Robot();

    public WebcamName webcamName;
    public OpenCvWebcam webcam;

    public enum ThisPosition {
        LEFT_POSITION,
        MIDDLE_POSITION,
        RIGHT_POSITION
    }

    public volatile ThisPosition WhatPosition;

    boolean firstTime = true;
    boolean runFSM = false;

    boolean isDeployed = false;

    double distance1 = 800;
    double angle1 = -60;
    double distance2 = 650;
    double angle2 = -90;
    double distance3 = 1845;

    @Override
    public void runOpMode() throws InterruptedException {

        r.telemetry = telemetry;
        r.init(hardwareMap);
        r.liftBox();
        r.updateAll();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

//        xPos = 0.0;
//        yPos = 0.0;
//        thetaPos = 0.0; //ofc might want to change

        sleep(150);

        while (!opModeIsActive())
            updateBoxPosition();

        waitForStart();

        if (isStopRequested()) return;

        r.capper.setPosition(CAP_HIGH);

        LeftRedState = LeftRed.FORWARD;
        MiddleRedState = MiddleRed.FORWARD;
        RightRedState = RightRed.FORWARD;

        r.autonWaitTimer.reset();
        //r.odoTimer.reset();
        webcam.stopStreaming();

        while (opModeIsActive()) {
            r.clearCache();

            switch (WhatPosition) {
                case LEFT_POSITION:
                    switch (LeftRedState) {
                        case FORWARD:
                            r.gyroStraight(-.5, 0);
                            if (Math.abs(r.backLeftPosition()) >= Math.abs(distance1) ||
                                    Math.abs(r.frontLeftPosition()) >= Math.abs(distance1) ||
                                    Math.abs(r.backRightPosition()) >= Math.abs(distance1) ||
                                    Math.abs(r.frontRightPosition()) >= Math.abs(distance1)) {
                                r.setTankPowers(0.0, 0.0);
                                r.autonWaitTimer.reset();
                                LeftRedState = LeftRed.TURN;
                            }
                            break;
                        case TURN:
                            r.setTankPowers(-(r.getAngle() - angle1) * .012, (r.getAngle() - angle1) * .012);

                            if (Math.abs(r.getAngle()) >= Math.abs(angle1) || r.autonWaitTimer.time() >= 3500) {
                                r.setTankPowers(0, 0);
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                LeftRedState = LeftRed.EXTEND;
                            }
                            break;
                        case EXTEND:
                            if (r.autonWaitTimer.time() >= 250 && runFSM) {
                                r.deployShared();
                                runFSM = false;
                                isDeployed = true;
                            }
                            if (r.autonWaitTimer.time() >= 750) {
                                r.linkageAdjust(.1);
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                LeftRedState = LeftRed.DROP;
                            }
                            break;
                        case DROP:
                            if (r.autonWaitTimer.time() >= 1000 && runFSM) {
                                r.dropoffBox();
                                if (r.autonWaitTimer.time() >= 1750 && firstTime) {
                                    r.linkageAdjust(-.4);
                                    firstTime = false;
                                }
                                if (r.autonWaitTimer.time() >= 2650){
                                    r.deployRest();
                                    runFSM = false;
                                    r.autonWaitTimer.reset();
                                    r.resetWheels();
                                    LeftRedState = LeftRed.FORWARD2;
                                }
                            }
                            break;
                        case FORWARD2:
                            break;
                        case TURN2:
                            break;
                        case PARK:
                            break;
                        case FINISH:
                            break;
                    }
                    break;

                case RIGHT_POSITION:
                    switch (RightRedState) {
                        case FORWARD:
                            r.gyroStraight(-.5, 0);
                            if (Math.abs(r.backLeftPosition()) >= Math.abs(distance1) ||
                                    Math.abs(r.frontLeftPosition()) >= Math.abs(distance1) ||
                                    Math.abs(r.backRightPosition()) >= Math.abs(distance1) ||
                                    Math.abs(r.frontRightPosition()) >= Math.abs(distance1)) {
                                r.setTankPowers(0.0, 0.0);
                                r.autonWaitTimer.reset();
                                RightRedState = RightRed.TURN;
                            }
                            break;
                        case TURN:
                            r.setTankPowers(-(r.getAngle() - angle1) * .012, (r.getAngle() - angle1) * .012);

                            if (Math.abs(r.getAngle()) >= Math.abs(angle1) || r.autonWaitTimer.time() >= 3500) {
                                r.setTankPowers(0, 0);
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                RightRedState = RightRed.EXTEND;
                            }
                            break;
                        case EXTEND:
                            if (r.autonWaitTimer.time() >= 250 && runFSM) {
                                r.deployTop();
                                r.linkageAdjust(-.3);
                                runFSM = false;
                                isDeployed = true;
                            }
                            if (r.autonWaitTimer.time() >= 1250) {
                                r.linkageAdjust(.34);
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                firstTime = true;
                                RightRedState = RightRed.DROP;
                            }
                            break;
                        case DROP:
                            if (r.autonWaitTimer.time() >= 1000 && runFSM) {
                                r.dropoffBox();
                                if (r.autonWaitTimer.time() >= 1750 && firstTime) {
                                    r.linkageAdjust(-.4);
                                    firstTime = false;
                                }
                                if (r.autonWaitTimer.time() >= 2650){
                                    r.deployRest();
                                    runFSM = false;
                                    r.autonWaitTimer.reset();
                                    r.resetWheels();
                                    RightRedState = RightRed.FORWARD2;
                                }
                            }
                            break;
                        case FORWARD2:
                            break;
                        case TURN2:
                            break;
                        case PARK:
                            break;
                        case FINISH:
                            break;
                    }
                    break;

                case MIDDLE_POSITION:
                    switch (MiddleRedState) {
                        case FORWARD:
                            r.gyroStraight(-.5, 0);
                            if (Math.abs(r.backLeftPosition()) >= Math.abs(distance1) ||
                                    Math.abs(r.frontLeftPosition()) >= Math.abs(distance1) ||
                                    Math.abs(r.backRightPosition()) >= Math.abs(distance1) ||
                                    Math.abs(r.frontRightPosition()) >= Math.abs(distance1)) {
                                r.setTankPowers(0.0, 0.0);
                                r.autonWaitTimer.reset();
                                MiddleRedState = MiddleRed.TURN;
                            }
                            break;
                        case TURN:
                            r.setTankPowers(-(r.getAngle() - angle1) * .012, (r.getAngle() - angle1) * .012);

                            if (Math.abs(r.getAngle()) >= Math.abs(angle1) || r.autonWaitTimer.time() >= 3500) {
                                r.setTankPowers(0, 0);
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                MiddleRedState = MiddleRed.EXTEND;
                            }
                            break;
                        case EXTEND:
                            if (r.autonWaitTimer.time() >= 250 && runFSM) {
                                r.deployMiddle();
                                r.slidesAdjust(-10);
                                r.linkageAdjust(-.3);
                                runFSM = false;
                                isDeployed = true;
                            }
                            if (r.autonWaitTimer.time() >= 1250) {
                                r.linkageAdjust(.38);
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                firstTime = true;
                                MiddleRedState = MiddleRed.DROP;
                            }
                            break;
                        case DROP:
                            if (r.autonWaitTimer.time() >= 1000 && runFSM) {
                                r.dropoffBox();
                                if (r.autonWaitTimer.time() >= 1750 && firstTime) {
                                    r.linkageAdjust(-.4);
                                    firstTime = false;
                                }
                                if (r.autonWaitTimer.time() >= 2650){
                                    r.deployRest();
                                    runFSM = false;
                                    r.autonWaitTimer.reset();
                                    r.resetWheels();
                                    MiddleRedState = MiddleRed.FORWARD2;
                                }
                            }
                            break;
                        case FORWARD2:
                            break;
                        case TURN2:
                            break;
                        case PARK:
                            break;
                        case FINISH:
                            break;
                    }
                    break;
            }

            r.updateAll();
//
//            telemetry.addData("x", r.getX());
//            telemetry.addData("y", r.getY());
//            telemetry.addData("heading", r.getTheta());
            telemetry.addData("angle", r.getAngle());
            telemetry.update();
        }
    }

    public void updateBoxPosition() {
        if (pipeline.position == null) {
            telemetry.addData("still working on it", "gimme a sec");
        } else if (pipeline.position == BoxPositionDetection.BoxPosition.RIGHT){
            telemetry.addData("Right Barcode, Top Level", "Waiting for start");
            WhatPosition = ThisPosition.RIGHT_POSITION;
        } else if (pipeline.position == BoxPositionDetection.BoxPosition.MIDDLE){
            telemetry.addData("Middle Barcode, Middle Level", "Waiting for start");
            WhatPosition = ThisPosition.MIDDLE_POSITION;
        } else if (pipeline.position == BoxPositionDetection.BoxPosition.LEFT){
            telemetry.addData("Left Barcode, Bottom Level", "Waiting for start");
            WhatPosition = ThisPosition.LEFT_POSITION;
        }
        telemetry.addData("average 1", avg1);
        telemetry.addData("average 2", avg2);
        telemetry.addData("average 3", avg3);
        telemetry.update();
        sleep(75);
    }
}
