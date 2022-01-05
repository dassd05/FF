package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.opencv.core.Point;

import java.util.*;

@Config
public class Constants {

    //TODO: servo positions
    public static double BOX_ROTATION_DOWN = 0.17;
    public static double BOX_ROTATION_UP = 0.56;
    public static double BOX_ROTATION_DEPLOY = 0.95;

    public static double ROTATE_TIME = 500;

    //TODO: min and max positions to avoid going too far
    public static int SLIDES_MAX = 1500;
    public static int SLIDES_MIN = 0;
    public static int SLIDES_TOP = 1000;
    public static int SLIDES_MID = 530;
    public static int SLIDES_LOW = 137;

    public static double LINKAGE_1_MIN = 0.0;
    public static double LINKAGE_1_MAX = 1.0;
    public static double LINKAGE_2_MIN = 0.0;
    public static double LINKAGE_2_MAX = 0.0;

    //tune
    public static double LINKAGE_SAFE_DROP = 0.2;
    public static double LINKAGE_SAFE_EXTEND = 120;

    //TODO: fix these adjustments as necessary
    public static double LINKAGE_ADJUSTMENT = .0025;
    public static int SLIDES_ADJUSTMENT = 2;

    public static double INTAKE_POWER = 0.9;

    public static PIDCoefficients pidConsts = new PIDCoefficients(0.0, 0.000, 0.0);
    public static PIDCoefficients pidConstsSlides = new PIDCoefficients(0.0045, 0.00000000, 0.0);
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
