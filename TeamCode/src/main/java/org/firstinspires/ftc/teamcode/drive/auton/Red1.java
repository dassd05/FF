package org.firstinspires.ftc.teamcode.drive.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.drive.auton.BaseAuton;

import static org.firstinspires.ftc.teamcode.drive.Robot.*;

@Autonomous(group = "A", name = "Red Left", preselectTeleOp = "RedTele")
public class Red1 extends BaseAuton {


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
}
