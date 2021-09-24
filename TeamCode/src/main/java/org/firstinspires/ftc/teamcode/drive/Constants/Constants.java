package org.firstinspires.ftc.teamcode.drive.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class Constants {

    public static PIDCoefficients pidConsts = new PIDCoefficients(0.4, 0.004, 0);
}
