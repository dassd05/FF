package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceAccuracy extends LinearOpMode {

    public Rev2mDistanceSensor sensor;

    @Override
    public void runOpMode() {

         sensor = hardwareMap.get(Rev2mDistanceSensor.class, "sensor");
         ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

         int cycles = 0;
         double distance = 0;

         waitForStart();
         timer.reset();

         if (opModeIsActive()) {
             while (opModeIsActive()) {
                 if (timer.time() > 15) {
                     distance = sensor.getDistance(DistanceUnit.CM);
                 }
                 cycles++;
                 telemetry.addData("distance", distance);
                 telemetry.addData("cycles per millisecond", cycles / timer.time());
                 telemetry.update();
             }
         }
    }
}
