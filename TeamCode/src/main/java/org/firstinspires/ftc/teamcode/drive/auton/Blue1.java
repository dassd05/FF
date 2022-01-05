package org.firstinspires.ftc.teamcode.drive.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.drive.auton.BaseAuton;

import static org.firstinspires.ftc.teamcode.drive.Robot.*;

@Autonomous(group = "A", name = "Blue Left", preselectTeleOp = "BlueTele")
public class Blue1 extends BaseAuton {

    @Override
    public void setRobotPosition() {
        robot.setPosition(0, 0, 0); //ofc might want to change
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        while (opModeIsActive()) {

            switch (boxPosition) {
                case LEFT:
                    switch (LeftBlueState) {

                    }
                    break;

                case RIGHT:
                    switch (RightBlueState) {

                    }
                    break;

                case MIDDLE:
                    switch (MiddleBlueState) {

                    }
                    break;
            }

            robot.updateAllStates();
            robot.updatePos(robot.getX(), robot.getY(), robot.getTheta(), robot.odoTimer.time());

            telemetry.addData("x", robot.getX());
            telemetry.addData("y", robot.getY());
            telemetry.addData("heading", robot.getTheta());
            telemetry.update();
        }
    }

//    public void updateBoxPosition() {
//        if (pipeline.position == null) {
//            telemetry.addData("still working on it", "gimme a sec");
//        } else if (pipeline.position == BoxPositionDetection.BoxPosition.RIGHT){
//            telemetry.addData("Right Position", "Waiting for start");
//            WhatPosition = ThisPosition.RIGHT_POSITION;
//            telemetry.update();
//        } else if (pipeline.position == BoxPositionDetection.BoxPosition.MIDDLE){
//            telemetry.addData("Middle Position", "Waiting for start");
//            WhatPosition = ThisPosition.MIDDLE_POSITION;
//        } else if (pipeline.position == BoxPositionDetection.BoxPosition.LEFT){
//            telemetry.addData("Left Position", "Waiting for start");
//            WhatPosition = ThisPosition.LEFT_POSITION;
//        }
//        telemetry.update();
//        sleep(75); //so we don't burn cpu cycles
//    }
}
