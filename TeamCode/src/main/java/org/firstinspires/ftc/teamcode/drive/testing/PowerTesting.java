package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.List;

@TeleOp(group="testing")
public class PowerTesting extends LinearOpMode {

    private List<VoltageSensor> voltageSensors;
    private List<LynxModule> lynxModules;
    private List<DcMotorEx> motors;


    @Override
    public void runOpMode() throws InterruptedException {
        voltageSensors = hardwareMap.getAll(VoltageSensor.class);
        lynxModules = hardwareMap.getAll(LynxModule.class);
        motors = hardwareMap.getAll(DcMotorEx.class);

        waitForStart();

        while (opModeIsActive()) {
            for (VoltageSensor voltageSensor : voltageSensors) {
                telemetry.addData(voltageSensor.getDeviceName(), voltageSensor.getVoltage());
            }
            for (LynxModule lynxModule : lynxModules) {
                String data = lynxModule.getInputVoltage(VoltageUnit.MILLIVOLTS) + " input voltage | "
                        + lynxModule.getAuxiliaryVoltage(VoltageUnit.MILLIVOLTS) + " auxiliary voltage | "
                        + lynxModule.getCurrent(CurrentUnit.MILLIAMPS) + " current";
                telemetry.addData(lynxModule.getDeviceName(), data);
            }
            for (DcMotorEx motor : motors) {
                telemetry.addData(motor.getDeviceName(), motor.getCurrent(CurrentUnit.MILLIAMPS));
            }
        }
    }
}
