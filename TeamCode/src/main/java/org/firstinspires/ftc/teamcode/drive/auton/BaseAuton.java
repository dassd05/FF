package org.firstinspires.ftc.teamcode.drive.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.vision.BoxDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.Robot;

import static org.firstinspires.ftc.teamcode.drive.vision.BoxDetectionPipeline.BoxPosition;

public abstract class BaseAuton extends LinearOpMode {
    public Robot robot;
    public BoxDetectionPipeline pipeline;

    public BoxPosition boxPosition;
    public boolean firstTime = true;
    public boolean runFSM = false;

    public ElapsedTime autonWaitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        pipeline = new BoxDetectionPipeline();
        robot.init();
        robot.dashboardInit();
        robot.webcamInit(pipeline);

        setRobotPosition();

        sleep(150); // just to make sure webcam is properly initialized before updateBoxPosition()

        while (!opModeIsActive()) updateBoxPosition();

        waitForStart();

        autonWaitTimer.reset();
        robot.odoTimer.reset();
        robot.webcam.stopStreaming();
    }

    public void updateBoxPosition() {
        //todo clean this up
        if (pipeline.position == null) {
            telemetry.addData("still working on it", "gimme a sec");
        } else if (pipeline.position == BoxPosition.RIGHT){
            telemetry.addData("Right Position", "Waiting for start");
            boxPosition = BoxPosition.RIGHT;
            telemetry.update();
        } else if (pipeline.position == BoxPosition.MIDDLE){
            telemetry.addData("Middle Position", "Waiting for start");
            boxPosition = BoxPosition.MIDDLE;
        } else if (pipeline.position == BoxPosition.LEFT){
            telemetry.addData("Left Position", "Waiting for start");
            boxPosition = BoxPosition.LEFT;
        }
        telemetry.update();
        sleep(75); //so we don't burn cpu cycles
    }

    public abstract void setRobotPosition();
}
