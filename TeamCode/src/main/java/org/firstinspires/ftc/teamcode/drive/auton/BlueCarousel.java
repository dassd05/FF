package org.firstinspires.ftc.teamcode.drive.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.Robot;

@Autonomous(group = "1", name = "\uD83D\uDD35 Carousel", preselectTeleOp = "Teleop")
public class BlueCarousel extends BaseAuton {

    Robot robot;

    boolean firstTime = true;
    boolean runFSM = false;

    boolean isDeployed = false;

    double distance1 = 800;
    double angle1 = -60;
    double distance2 = 1100;
    double angle1_2 = -52;
    double angle2 = 0;
    double distance3 = 100;
    double distance4 = 810;

    @Override
    public void setRobotPosition() {
//        robot.setPosition(0, 0, 0); //ofc might want to change
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        robot.capper.setPosition(Constants.CAP_HIGH);

        LeftRedState = LeftRed.FORWARD;
        MiddleRedState = MiddleRed.FORWARD;
        RightRedState = RightRed.FORWARD;


        while (opModeIsActive()) {
            robot.clearCache();

            switch (boxPosition) {
                case LEFT:
                    switch (LeftRedState) {
                        case FORWARD:
                            robot.gyroStraight(-.5, 0);
                            if (Math.abs(robot.backLeftPosition()) >= Math.abs(distance1) ||
                                    Math.abs(robot.frontLeftPosition()) >= Math.abs(distance1) ||
                                    Math.abs(robot.backRightPosition()) >= Math.abs(distance1) ||
                                    Math.abs(robot.frontRightPosition()) >= Math.abs(distance1)) {
                                robot.setTankPowers(0.0, 0.0);
                                autonWaitTimer.reset();
                                LeftRedState = LeftRed.TURN;
                            }
                            break;
                        case TURN:
                            robot.setTankPowers(-(robot.getAngle() - angle1) * .0095, (robot.getAngle() - angle1) * .0095);

                            if (Math.abs(robot.getAngle()) >= Math.abs(angle1) || autonWaitTimer.time() >= 3500) {
                                robot.setTankPowers(0, 0);
                                autonWaitTimer.reset();
                                runFSM = true;
                                LeftRedState = LeftRed.EXTEND;
                            }
                            break;
                        case EXTEND:
                            if (autonWaitTimer.time() >= 250 && runFSM) {
                                robot.deployShared();
                                robot.linkageAdjust(-.3);
                                runFSM = false;
                                isDeployed = true;
                            }
                            if (autonWaitTimer.time() >= 1250) {
                                robot.linkageAdjust(.42);
                                autonWaitTimer.reset();
                                runFSM = true;
                                firstTime = true;
                                LeftRedState = LeftRed.DROP;
                            }
                            break;
                        case DROP:
                            if (autonWaitTimer.time() >= 1000 && runFSM) {
                                robot.boxDrop();
                                if (autonWaitTimer.time() >= 1750 && firstTime) {
                                    robot.linkageAdjust(-.4);
                                    firstTime = false;
                                }
                                if (autonWaitTimer.time() >= 2650) {
                                    robot.deployRest();
                                    runFSM = false;
                                    autonWaitTimer.reset();
                                    robot.resetWheels();
                                    LeftRedState = LeftRed.FORWARD2;
                                }
                            }
                            break;
                        case FORWARD2:
                            robot.gyroStraight(.25, angle1_2);
                            if (Math.abs(robot.backLeftPosition()) >= Math.abs(distance2) ||
                                    Math.abs(robot.frontLeftPosition()) >= Math.abs(distance2) ||
                                    Math.abs(robot.backRightPosition()) >= Math.abs(distance2) ||
                                    Math.abs(robot.frontRightPosition()) >= Math.abs(distance2) ||
                                    autonWaitTimer.time() >= 5000) {
                                robot.setTankPowers(0.1, 0.1);
                                autonWaitTimer.reset();
                                LeftRedState = LeftRed.TURN2;
                            }
                            break;
                        case TURN2:
                            robot.carousel.setPower(.4);
                            if (autonWaitTimer.time() > 6500) {
                                robot.resetWheels();
                                autonWaitTimer.reset();
                                robot.carousel.setPower(0);
                                LeftRedState = LeftRed.PARK;
                            }
                            break;
                        case PARK:
                            robot.gyroStraight(-.2, 0);
                            if (Math.abs(robot.backLeftPosition()) >= Math.abs(distance3) ||
                                    Math.abs(robot.frontLeftPosition()) >= Math.abs(distance3) ||
                                    Math.abs(robot.backRightPosition()) >= Math.abs(distance3) ||
                                    Math.abs(robot.frontRightPosition()) >= Math.abs(distance3)) {
                                robot.setTankPowers(0.0, 0.0);
                                autonWaitTimer.reset();
                                LeftRedState = LeftRed.FINISH;
                            }
                            break;
                        case FINISH:
                            robot.setTankPowers(-(robot.getAngle() - angle2) * .0095, (robot.getAngle() - angle2) * .0095);

                            if (Math.abs(robot.getAngle()) >= angle2 || autonWaitTimer.time() >= 3500) {
                                robot.setTankPowers(0, 0);
                                robot.resetWheels();
                                autonWaitTimer.reset();
                                runFSM = true;
                                LeftRedState = LeftRed.CAROUSEL;
                            }
                            break;
                        case CAROUSEL:
                            robot.gyroStraight(-.2, 0);
                            if (Math.abs(robot.backLeftPosition()) >= Math.abs(distance4) ||
                                    Math.abs(robot.frontLeftPosition()) >= Math.abs(distance4) ||
                                    Math.abs(robot.backRightPosition()) >= Math.abs(distance4) ||
                                    Math.abs(robot.frontRightPosition()) >= Math.abs(distance4)) {
                                robot.setTankPowers(0.0, 0.0);
                                autonWaitTimer.reset();
                                LeftRedState = LeftRed.ACTUAL_FINISH;
                            }
                            break;
                        case ACTUAL_FINISH:
                            robot.setTankPowers(0, 0);
                            break;
                    }
                    telemetry.addData("state", LeftRedState);
                    break;

                case RIGHT:
                    switch (RightRedState) {
                        case FORWARD:
                            robot.gyroStraight(-.5, 0);
                            if (Math.abs(robot.backLeftPosition()) >= Math.abs(distance1) ||
                                    Math.abs(robot.frontLeftPosition()) >= Math.abs(distance1) ||
                                    Math.abs(robot.backRightPosition()) >= Math.abs(distance1) ||
                                    Math.abs(robot.frontRightPosition()) >= Math.abs(distance1)) {
                                robot.setTankPowers(0.0, 0.0);
                                autonWaitTimer.reset();
                                RightRedState = RightRed.TURN;
                            }
                            break;
                        case TURN:
                            robot.setTankPowers(-(robot.getAngle() - angle1) * .0095, (robot.getAngle() - angle1) * .0095);

                            if (Math.abs(robot.getAngle()) >= Math.abs(angle1) || autonWaitTimer.time() >= 3500) {
                                robot.setTankPowers(0, 0);
                                autonWaitTimer.reset();
                                runFSM = true;
                                RightRedState = RightRed.EXTEND;
                            }
                            break;
                        case EXTEND:
                            if (autonWaitTimer.time() >= 250 && runFSM) {
                                robot.deployTop();
                                robot.linkageAdjust(-.3);
                                runFSM = false;
                                isDeployed = true;
                            }
                            if (autonWaitTimer.time() >= 1250) {
                                robot.linkageAdjust(.34);
                                autonWaitTimer.reset();
                                runFSM = true;
                                firstTime = true;
                                RightRedState = RightRed.DROP;
                            }
                            break;
                        case DROP:
                            if (autonWaitTimer.time() >= 1000 && runFSM) {
                                robot.boxDrop();
                                if (autonWaitTimer.time() >= 1750 && firstTime) {
                                    robot.linkageAdjust(-.4);
                                    firstTime = false;
                                }
                                if (autonWaitTimer.time() >= 2650) {
                                    robot.deployRest();
                                    runFSM = false;
                                    autonWaitTimer.reset();
                                    robot.resetWheels();
                                    RightRedState = RightRed.FORWARD2;
                                }
                            }
                            break;
                        case FORWARD2:
                            robot.gyroStraight(.25, angle1_2);
                            if (Math.abs(robot.backLeftPosition()) >= Math.abs(distance2) ||
                                    Math.abs(robot.frontLeftPosition()) >= Math.abs(distance2) ||
                                    Math.abs(robot.backRightPosition()) >= Math.abs(distance2) ||
                                    Math.abs(robot.frontRightPosition()) >= Math.abs(distance2) ||
                                    autonWaitTimer.time() >= 5000) {
                                robot.setTankPowers(0.1, 0.1);
                                autonWaitTimer.reset();
                                RightRedState = RightRed.TURN2;
                            }
                            break;
                        case TURN2:
                            robot.carousel.setPower(.4);
                            if (autonWaitTimer.time() > 6500) {
                                robot.resetWheels();
                                autonWaitTimer.reset();
                                robot.carousel.setPower(0);
                                RightRedState = RightRed.PARK;
                            }
                            break;
                        case PARK:
                            robot.gyroStraight(-.2, 0);
                            if (Math.abs(robot.backLeftPosition()) >= Math.abs(distance3) ||
                                    Math.abs(robot.frontLeftPosition()) >= Math.abs(distance3) ||
                                    Math.abs(robot.backRightPosition()) >= Math.abs(distance3) ||
                                    Math.abs(robot.frontRightPosition()) >= Math.abs(distance3)) {
                                robot.setTankPowers(0.0, 0.0);
                                autonWaitTimer.reset();
                                RightRedState = RightRed.FINISH;
                            }
                            break;
                        case FINISH:
                            robot.setTankPowers(-(robot.getAngle() - angle2) * .0095, (robot.getAngle() - angle2) * .0095);

                            if (Math.abs(robot.getAngle()) >= angle2 || autonWaitTimer.time() >= 3500) {
                                robot.setTankPowers(0, 0);
                                robot.resetWheels();
                                autonWaitTimer.reset();
                                runFSM = true;
                                RightRedState = RightRed.CAROUSEL;
                            }
                            break;
                        case CAROUSEL:
                            robot.gyroStraight(-.2, 0);
                            if (Math.abs(robot.backLeftPosition()) >= Math.abs(distance4) ||
                                    Math.abs(robot.frontLeftPosition()) >= Math.abs(distance4) ||
                                    Math.abs(robot.backRightPosition()) >= Math.abs(distance4) ||
                                    Math.abs(robot.frontRightPosition()) >= Math.abs(distance4)) {
                                robot.setTankPowers(0.0, 0.0);
                                autonWaitTimer.reset();
                                RightRedState = RightRed.ACTUAL_FINISH;
                            }
                            break;
                        case ACTUAL_FINISH:
                            robot.setTankPowers(0, 0);
                            break;
                    }
                    telemetry.addData("state", RightRedState);
                    break;

                case MIDDLE:
                    switch (MiddleRedState) {
                        case FORWARD:
                            robot.gyroStraight(-.5, 0);
                            if (Math.abs(robot.backLeftPosition()) >= Math.abs(distance1) ||
                                    Math.abs(robot.frontLeftPosition()) >= Math.abs(distance1) ||
                                    Math.abs(robot.backRightPosition()) >= Math.abs(distance1) ||
                                    Math.abs(robot.frontRightPosition()) >= Math.abs(distance1)) {
                                robot.setTankPowers(0.0, 0.0);
                                autonWaitTimer.reset();
                                MiddleRedState = MiddleRed.TURN;
                            }
                            break;
                        case TURN:
                            robot.setTankPowers(-(robot.getAngle() - angle1) * .0095, (robot.getAngle() - angle1) * .0095);

                            if (Math.abs(robot.getAngle()) >= Math.abs(angle1) || autonWaitTimer.time() >= 3500) {
                                robot.setTankPowers(0, 0);
                                autonWaitTimer.reset();
                                runFSM = true;
                                MiddleRedState = MiddleRed.EXTEND;
                            }
                            break;
                        case EXTEND:
                            if (autonWaitTimer.time() >= 250 && runFSM) {
                                robot.deployMiddle();
                                robot.slidesAdjust(-10);
                                robot.linkageAdjust(-.3);
                                runFSM = false;
                                isDeployed = true;
                            }
                            if (autonWaitTimer.time() >= 1250) {
                                robot.linkageAdjust(.38);
                                autonWaitTimer.reset();
                                runFSM = true;
                                firstTime = true;
                                MiddleRedState = MiddleRed.DROP;
                            }
                            break;
                        case DROP:
                            if (autonWaitTimer.time() >= 1000 && runFSM) {
                                robot.boxDrop();
                                if (autonWaitTimer.time() >= 1750 && firstTime) {
                                    robot.linkageAdjust(-.4);
                                    firstTime = false;
                                }
                                if (autonWaitTimer.time() >= 2650) {
                                    robot.deployRest();
                                    runFSM = false;
                                    autonWaitTimer.reset();
                                    robot.resetWheels();
                                    MiddleRedState = MiddleRed.FORWARD2;
                                }
                            }
                            break;
                        case FORWARD2:
                            robot.gyroStraight(.25, angle1_2);
                            if (Math.abs(robot.backLeftPosition()) >= Math.abs(distance2) ||
                                    Math.abs(robot.frontLeftPosition()) >= Math.abs(distance2) ||
                                    Math.abs(robot.backRightPosition()) >= Math.abs(distance2) ||
                                    Math.abs(robot.frontRightPosition()) >= Math.abs(distance2) ||
                                    autonWaitTimer.time() >= 5000) {
                                robot.setTankPowers(0.1, 0.1);
                                autonWaitTimer.reset();
                                MiddleRedState = MiddleRed.TURN2;
                            }
                            break;
                        case TURN2:
                            robot.carousel.setPower(.4);
                            if (autonWaitTimer.time() > 6500) {
                                robot.resetWheels();
                                autonWaitTimer.reset();
                                robot.carousel.setPower(0);
                                MiddleRedState = MiddleRed.PARK;
                            }
                            break;
                        case PARK:
                            robot.gyroStraight(-.2, 0);
                            if (Math.abs(robot.backLeftPosition()) >= Math.abs(distance3) ||
                                    Math.abs(robot.frontLeftPosition()) >= Math.abs(distance3) ||
                                    Math.abs(robot.backRightPosition()) >= Math.abs(distance3) ||
                                    Math.abs(robot.frontRightPosition()) >= Math.abs(distance3)) {
                                robot.setTankPowers(0.0, 0.0);
                                autonWaitTimer.reset();
                                MiddleRedState = MiddleRed.FINISH;
                            }
                            break;
                        case FINISH:
                            robot.setTankPowers(-(robot.getAngle() - angle2) * .0095, (robot.getAngle() - angle2) * .0095);

                            if (Math.abs(robot.getAngle()) >= angle2 || autonWaitTimer.time() >= 3500) {
                                robot.setTankPowers(0, 0);
                                robot.resetWheels();
                                autonWaitTimer.reset();
                                runFSM = true;
                                MiddleRedState = MiddleRed.CAROUSEL;
                            }
                            break;
                        case CAROUSEL:
                            robot.gyroStraight(-.2, 0);
                            if (Math.abs(robot.backLeftPosition()) >= Math.abs(distance4) ||
                                    Math.abs(robot.frontLeftPosition()) >= Math.abs(distance4) ||
                                    Math.abs(robot.backRightPosition()) >= Math.abs(distance4) ||
                                    Math.abs(robot.frontRightPosition()) >= Math.abs(distance4)) {
                                robot.setTankPowers(0.0, 0.0);
                                autonWaitTimer.reset();
                                MiddleRedState = MiddleRed.ACTUAL_FINISH;
                            }
                            break;
                        case ACTUAL_FINISH:
                            robot.setTankPowers(0, 0);
                            break;
                    }
                    telemetry.addData("state", MiddleRedState);
                    break;
            }

            robot.updateAll();
//
//            telemetry.addData("x", robot.getX());
//            telemetry.addData("y", robot.getY());
//            telemetry.addData("heading", robot.getTheta());
            telemetry.addData("angle", robot.getAngle());
            telemetry.update();
        }
    }

}
