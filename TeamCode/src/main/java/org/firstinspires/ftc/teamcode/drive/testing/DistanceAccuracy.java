package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceAccuracy extends LinearOpMode {

    public Rev2mDistanceSensor sensor;

    @Override
    public void runOpMode() {
         hardwareMap.logDevices();

         //sensor = hardwareMap.get(Rev2mDistanceSensor.class, "sensor");
         sensor = hardwareMap.getAll(Rev2mDistanceSensor.class).get(0);

         waitForStart();

         while (opModeIsActive()) {
             telemetry.addData("distance", sensor.getDistance(DistanceUnit.MM));
             telemetry.update();
         }
    }
}
