package org.firstinspires.ftc.teamcode.drive.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import java.util.*;

@Config
public class Constants {

    public static PIDCoefficients pidConsts = new PIDCoefficients(0.0, 0.000, 0.0);

    public static double WHEEL_RADIUS = 0.0;
    public static double TICKS_PER_REV = 0.0;
    public static double GEAR_RATIO = 0.0;

    public static double trackWidth = 0.0;

    public static double LEFT_WHEEL_MULTIPLIER = 0.0;
    public static double RIGHT_WHEEL_MULTIPLIER = 0.0;
}
