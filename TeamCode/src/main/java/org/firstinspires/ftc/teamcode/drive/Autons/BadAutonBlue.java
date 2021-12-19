package org.firstinspires.ftc.teamcode.drive.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot;

@Deprecated
@Autonomous(name = "BadAutonBlue", group = "1")
public class BadAutonBlue extends LinearOpMode {

    Robot r = new Robot();
    ElapsedTime badTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public enum states {
        MOVE_FOR_CAROUSEL,
        CAROUSEL,
        MOVE_AWAY_CAROUSEL,
        TURN,
        WAIT,
        PARK,
        FINISH
    }
    public states autonState;

    @Override
    public void runOpMode() throws InterruptedException {

        r.telemetry = telemetry;
        r.init(hardwareMap);

        waitForStart();
        badTimer.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (badTimer.time() < 1000)
                    autonState = states.MOVE_FOR_CAROUSEL;
                else if (badTimer.time() < 10000)
                    autonState = states.CAROUSEL;
                else if (badTimer.time() < 12000)
                    autonState = states.MOVE_AWAY_CAROUSEL;
                else if (badTimer.time() < 14000)
                    autonState = states.TURN;
                else if (badTimer.time() < 23000)
                    autonState = states.WAIT;
                else if (badTimer.time() < 28000)
                    autonState = states.PARK;
                else
                    autonState = states.FINISH;

                switch (autonState) {
                    case MOVE_FOR_CAROUSEL:
                        break;

                    case CAROUSEL:
                        break;

                    case MOVE_AWAY_CAROUSEL:
                        break;

                    case TURN:
                        break;

                    case WAIT:
                        break;

                    case PARK:
                        break;

                    case FINISH:
                        break;
                }

                telemetry.update();
            }
        }
    }
}