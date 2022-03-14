package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.List;

public class HardwareTest extends LinearOpMode {

    List<HardwareDevice> hardware;

    public static boolean showAll = true;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = hardwareMap.getAll(HardwareDevice.class);

        hardwareMap.logDevices();

        telemetry.setItemSeparator("-");

        while (opModeIsActive()) {
            if (showAll) {
                for (HardwareDevice ware : hardware) {
                    telemetry.addData(ware.getClass().toString(), ware.getDeviceName() + " - " +  ware.getConnectionInfo());

                }
            }

        }
    }
}
