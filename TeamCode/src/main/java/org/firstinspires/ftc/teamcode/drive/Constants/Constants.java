package org.firstinspires.ftc.teamcode.drive.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.opencv.core.Point;

import java.util.*;

@Config
public class Constants {

    public static PIDCoefficients pidConsts = new PIDCoefficients(0.0, 0.000, 0.0);
    //TODO: tune PID

    public static double WHEEL_RADIUS = 0.0;
    public static double TICKS_PER_REV = 0.0;
    public static double GEAR_RATIO = 0.0;

    public static double trackWidth = 0.0;

    public static double LEFT_WHEEL_MULTIPLIER = 0.0;
    public static double RIGHT_WHEEL_MULTIPLIER = 0.0;
    //TODO: tune odo constants

    //box detecting constants
    public static double THRESHOLD = 0.0;

    public static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(500, 500);
    public static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(750, 500);
    public static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1000, 500);


    public static final int REGION_WIDTH = 100;
    public static final int REGION_HEIGHT = 100;
    //TODO: fix all the box detecting constants
}
