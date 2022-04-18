package org.firstinspires.ftc.teamcode.drive.Autons.Red;

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
import static org.firstinspires.ftc.teamcode.drive.Robot.*;

@Autonomous(group = "1", name = "\uD83D\uDFE5 Carousel", preselectTeleOp = "Teleop")
public class RedCarousel extends LinearOpMode {

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
    double angle1_middle = 53;
    double angle1_regular = 57;
    double distance2 = 1100;
    double angle1_2 = 52;
    double angle2 = 0;
    double distance3 = 100;
    double distance4 = 835;

    @Override
    public void runOpMode() throws InterruptedException {

        r.telemetry = telemetry;
        r.init(hardwareMap);

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

        sleep(150);

        while (!opModeIsActive()) {
            r.freightLoaded = true;
            r.updateAll();
            updateBoxPosition();
        }

        waitForStart();

        if (isStopRequested()) return;

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
                            r.runPID = (Math.abs(r.getAngle()) < Math.abs(angle1_regular));

                            if (r.runPID && r.autonWaitTimer.time() < 3500)
                                r.turnPID(angle1_regular, r.turnTimer.time(), r.runPID);
                            else {
                                r.setTankPowers(0, 0);
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                r.runPID = false;
                                LeftRedState = LeftRed.EXTEND;
                            }

                            break;
                        case EXTEND:
                            if (r.autonWaitTimer.time() >= 250 && runFSM) {
                                r.deployBottom();
                                r.freightLoaded = false;
                                runFSM = false;
                                isDeployed = true;
                            }
                            if (r.autonWaitTimer.time() >= 2000) {
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                firstTime = true;
                                LeftRedState = LeftRed.DROP;
                            }
                            break;
                        case DROP:
                            if (r.autonWaitTimer.time() >= 600 && runFSM) {
                                r.deployAlliance();
                                if (r.autonWaitTimer.time() >= 1500 && firstTime) {
                                    r.linkageAdjust(-.2);
                                    firstTime = false;
                                }
                                if (r.autonWaitTimer.time() >= 1750) {
                                    r.deployRest();
                                    runFSM = false;
                                    r.autonWaitTimer.reset();
                                    r.resetWheels();
                                    LeftRedState = LeftRed.FORWARD2;
                                }
                            }
                            break;
                        case FORWARD2:
                            r.gyroStraight(.25, angle1_2);
                            if (Math.abs(r.backLeftPosition()) >= Math.abs(distance2) ||
                                    Math.abs(r.frontLeftPosition()) >= Math.abs(distance2) ||
                                    Math.abs(r.backRightPosition()) >= Math.abs(distance2) ||
                                    Math.abs(r.frontRightPosition()) >= Math.abs(distance2) ||
                                    r.autonWaitTimer.time() >= 5000) {
                                r.setTankPowers(0.1, 0.1);
                                r.autonWaitTimer.reset();
                                LeftRedState = LeftRed.TURN2;
                            }
                            break;
                        case TURN2:
                            r.carousel2.setPower(-1);
                            if (r.autonWaitTimer.time() > 6500) {
                                r.resetWheels();
                                r.autonWaitTimer.reset();
                                r.carousel2.setPower(0);
                                LeftRedState = LeftRed.PARK;
                            }
                            break;
                        case PARK:
                            r.gyroStraight(-.2, 0);
                            if (Math.abs(r.backLeftPosition()) >= Math.abs(distance3) ||
                                    Math.abs(r.frontLeftPosition()) >= Math.abs(distance3) ||
                                    Math.abs(r.backRightPosition()) >= Math.abs(distance3) ||
                                    Math.abs(r.frontRightPosition()) >= Math.abs(distance3)) {
                                r.setTankPowers(0.0, 0.0);
                                r.autonWaitTimer.reset();
                                LeftRedState = LeftRed.FINISH;
                            }
                            break;
                        case FINISH:
                            r.setTankPowers(-(r.getAngle() - angle2) * .0095, (r.getAngle() - angle2) * .0095);

                            if (Math.abs(r.getAngle()) >= angle2 || r.autonWaitTimer.time() >= 3500) {
                                r.setTankPowers(0, 0);
                                r.resetWheels();
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                LeftRedState = LeftRed.CAROUSEL;
                            }
                            break;
                        case CAROUSEL:
                            r.gyroStraight(-.2, 0);
                            if (Math.abs(r.backLeftPosition()) >= Math.abs(distance4) ||
                                    Math.abs(r.frontLeftPosition()) >= Math.abs(distance4) ||
                                    Math.abs(r.backRightPosition()) >= Math.abs(distance4) ||
                                    Math.abs(r.frontRightPosition()) >= Math.abs(distance4)) {
                                r.setTankPowers(0.0, 0.0);
                                r.autonWaitTimer.reset();
                                LeftRedState = LeftRed.ACTUAL_FINISH;
                            }
                            break;
                        case ACTUAL_FINISH:
                            r.setTankPowers(0,0);
                            break;
                    }
                    telemetry.addData("state", LeftRedState);
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
                            if (Math.abs(r.getAngle()) >= Math.abs(angle1_regular) || r.autonWaitTimer.time() >= 3500) {
                                r.runPID = false;
                                r.setTankPowers(0, 0);
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                RightRedState = RightRed.EXTEND;
                            } else {
                                r.runPID = true;
                                r.turnPID(angle1_regular, r.turnTimer.time(), r.runPID);
                            }
                            break;
                        case EXTEND:
                            if (r.autonWaitTimer.time() >= 250 && runFSM) {
                                r.deployTop();
                                r.freightLoaded = false;
                                runFSM = false;
                                isDeployed = true;
                            }
                            if (r.autonWaitTimer.time() >= 2000) {
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                firstTime = true;
                                RightRedState = RightRed.DROP;
                            }
                            break;
                        case DROP:

                            if (r.autonWaitTimer.time() >= 600 && runFSM) {
                                r.deployAlliance();
                                if (r.autonWaitTimer.time() >= 2500) {
                                    r.bringIn();
                                    r.deployRest();
                                    runFSM = false;
                                }
                            }

                            if (r.autonWaitTimer.time() >= 3500) {
                                r.autonWaitTimer.reset();
                                r.resetWheels();
                                RightRedState = RightRed.FORWARD2;
                            }

                            break;
                        case FORWARD2:
                            r.gyroStraight(.25, angle1_2);
                            if (Math.abs(r.backLeftPosition()) >= Math.abs(distance2) ||
                                    Math.abs(r.frontLeftPosition()) >= Math.abs(distance2) ||
                                    Math.abs(r.backRightPosition()) >= Math.abs(distance2) ||
                                    Math.abs(r.frontRightPosition()) >= Math.abs(distance2) ||
                                    r.autonWaitTimer.time() >= 5000) {
                                r.setTankPowers(0.1, 0.1);
                                r.autonWaitTimer.reset();
                                RightRedState = RightRed.TURN2;
                            }
                            break;
                        case TURN2:
                            r.carousel2.setPower(-1);
                            if (r.autonWaitTimer.time() > 6500) {
                                r.resetWheels();
                                r.autonWaitTimer.reset();
                                r.carousel2.setPower(0);
                                RightRedState = RightRed.PARK;
                            }
                            break;
                        case PARK:
                            r.gyroStraight(-.2, 0);
                            if (Math.abs(r.backLeftPosition()) >= Math.abs(distance3) ||
                                    Math.abs(r.frontLeftPosition()) >= Math.abs(distance3) ||
                                    Math.abs(r.backRightPosition()) >= Math.abs(distance3) ||
                                    Math.abs(r.frontRightPosition()) >= Math.abs(distance3)) {
                                r.setTankPowers(0.0, 0.0);
                                r.autonWaitTimer.reset();
                                RightRedState = RightRed.FINISH;
                            }
                            break;
                        case FINISH:
                            r.setTankPowers(-(r.getAngle() - angle2) * .0095, (r.getAngle() - angle2) * .0095);

                            if (Math.abs(r.getAngle()) >= angle2 || r.autonWaitTimer.time() >= 3500) {
                                r.setTankPowers(0, 0);
                                r.resetWheels();
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                RightRedState = RightRed.CAROUSEL;
                            }
                            break;
                        case CAROUSEL:
                            r.gyroStraight(-.2, 0);
                            if (Math.abs(r.backLeftPosition()) >= Math.abs(distance4) ||
                                    Math.abs(r.frontLeftPosition()) >= Math.abs(distance4) ||
                                    Math.abs(r.backRightPosition()) >= Math.abs(distance4) ||
                                    Math.abs(r.frontRightPosition()) >= Math.abs(distance4)) {
                                r.setTankPowers(0.0, 0.0);
                                r.autonWaitTimer.reset();
                                RightRedState = RightRed.ACTUAL_FINISH;
                            }
                            break;
                        case ACTUAL_FINISH:
                            r.setTankPowers(0,0);
                            break;
                    }
                    telemetry.addData("state", RightRedState);
                    break;

                case MIDDLE_POSITION:
                    switch (MiddleRedState) {
                        case FORWARD:
                            r.gyroStraight(-.3, 0);
                            if (Math.abs(r.backLeftPosition()) >= Math.abs(distance1 + 150) ||
                                    Math.abs(r.frontLeftPosition()) >= Math.abs(distance1 + 150) ||
                                    Math.abs(r.backRightPosition()) >= Math.abs(distance1 + 150) ||
                                    Math.abs(r.frontRightPosition()) >= Math.abs(distance1 + 150)) {
                                r.setTankPowers(0.0, 0.0);
                                r.autonWaitTimer.reset();
                                MiddleRedState = MiddleRed.BACK;
                            }
                            break;
                        case BACK:
                            r.gyroStraight(.4, 0);
                            if (Math.abs(r.backLeftPosition()) <= Math.abs(distance1) ||
                                    Math.abs(r.frontLeftPosition()) <= Math.abs(distance1) ||
                                    Math.abs(r.backRightPosition()) <= Math.abs(distance1) ||
                                    Math.abs(r.frontRightPosition()) <= Math.abs(distance1)) {
                                r.setTankPowers(0.0, 0.0);
                                r.autonWaitTimer.reset();
                                MiddleRedState = MiddleRed.TURN;
                            }
                            break;
                        case TURN:
                            r.runPID = (Math.abs(r.getAngle()) < Math.abs(angle1_middle));

                            if (r.runPID && r.autonWaitTimer.time() < 3500)
                                r.turnPID(angle1_middle, r.turnTimer.time(), r.runPID);
                            else {
                                r.setTankPowers(0, 0);
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                r.runPID = false;
                                MiddleRedState = MiddleRed.EXTEND;
                            }

                            break;
                        case EXTEND:
                            if (r.autonWaitTimer.time() >= 250 && runFSM) {
                                r.deployMiddle();
                                r.freightLoaded = false;
                                runFSM = false;
                                isDeployed = true;
                            }
                            if (r.autonWaitTimer.time() >= 2000) {
                                r.linkageAdjust(.3);
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                firstTime = true;
                                MiddleRedState = MiddleRed.DROP;
                            }
                            break;
                        case DROP:
                            if (r.autonWaitTimer.time() >= 600 && runFSM) {
                                r.deployAlliance();
                                if (r.autonWaitTimer.time() >= 2500) {
                                    r.bringIn();
                                    r.deployRest();
                                    runFSM = false;
                                }
                            }

                            if (r.autonWaitTimer.time() >= 3500){
                                r.autonWaitTimer.reset();
                                r.resetWheels();
                                MiddleRedState = MiddleRed.FORWARD2;
                            }
                            break;
                        case FORWARD2:
                            r.gyroStraight(.25, angle1_2 + 9);
                            if (Math.abs(r.backLeftPosition()) >= Math.abs(distance2) ||
                                    Math.abs(r.frontLeftPosition()) >= Math.abs(distance2) ||
                                    Math.abs(r.backRightPosition()) >= Math.abs(distance2) ||
                                    Math.abs(r.frontRightPosition()) >= Math.abs(distance2) ||
                                    r.autonWaitTimer.time() >= 5000) {
                                r.setTankPowers(0.1, 0.1);
                                r.autonWaitTimer.reset();
                                MiddleRedState = MiddleRed.TURN2;
                            }
                            break;
                        case TURN2:
                            r.carousel2.setPower(-1);
                            if (r.autonWaitTimer.time() > 6500) {
                                r.resetWheels();
                                r.autonWaitTimer.reset();
                                r.carousel2.setPower(0);
                                MiddleRedState = MiddleRed.PARK;
                            }
                            break;
                        case PARK:
                            r.gyroStraight(-.2, 0);
                            if (Math.abs(r.backLeftPosition()) >= Math.abs(distance3) ||
                                    Math.abs(r.frontLeftPosition()) >= Math.abs(distance3) ||
                                    Math.abs(r.backRightPosition()) >= Math.abs(distance3) ||
                                    Math.abs(r.frontRightPosition()) >= Math.abs(distance3)) {
                                r.setTankPowers(0.0, 0.0);
                                r.autonWaitTimer.reset();
                                MiddleRedState = MiddleRed.FINISH;
                            }
                            break;
                        case FINISH:
                            r.setTankPowers(-(r.getAngle() - angle2) * .0095, (r.getAngle() - angle2) * .0095);

                            if (Math.abs(r.getAngle()) >= angle2 || r.autonWaitTimer.time() >= 3500) {
                                r.setTankPowers(0, 0);
                                r.resetWheels();
                                r.autonWaitTimer.reset();
                                runFSM = true;
                                MiddleRedState = MiddleRed.CAROUSEL;
                            }
                            break;
                        case CAROUSEL:
                            r.gyroStraight(-.2, 0);
                            if (Math.abs(r.backLeftPosition()) >= Math.abs(distance4 + 15) ||
                                    Math.abs(r.frontLeftPosition()) >= Math.abs(distance4 + 15) ||
                                    Math.abs(r.backRightPosition()) >= Math.abs(distance4 + 15) ||
                                    Math.abs(r.frontRightPosition()) >= Math.abs(distance4) +15) {
                                r.setTankPowers(0.0, 0.0);
                                r.autonWaitTimer.reset();
                                MiddleRedState = MiddleRed.ACTUAL_FINISH;
                            }
                            break;
                        case ACTUAL_FINISH:
                            r.setTankPowers(0,0);
                            break;
                    }
                    telemetry.addData("state", MiddleRedState);
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
        switch (pipeline.position) {
            case MIDDLE:
                telemetry.addData("Middle Barcode, Middle Level", "Waiting for start");
                WhatPosition = ThisPosition.MIDDLE_POSITION;
                break;
            case LEFT:
                telemetry.addData("Left Barcode, Bottom Level", "Waiting for start");
                WhatPosition = ThisPosition.LEFT_POSITION;
                break;
            case RIGHT:
                telemetry.addData("Right Barcode, Top Level", "Waiting for start");
                WhatPosition = ThisPosition.RIGHT_POSITION;
                break;
            default:
                telemetry.addData("still working on it", "gimme a sec");
                break;
        }
        telemetry.addData("average 1", avg1);
        telemetry.addData("average 2", avg2);
        telemetry.addData("average 3", avg3);
        telemetry.update();
        sleep(75);
    }
}