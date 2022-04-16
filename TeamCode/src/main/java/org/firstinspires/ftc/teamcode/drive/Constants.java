package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.opencv.core.Point;

import java.util.*;

@Config
public class Constants {

    //TODO: servo positions
    public static double ARM_MAX_IN = .101;
    public static double ARM_VERTICAL = .425;
    public static double ARM_STRAIGHT_OUT = .777;
    public static double ARM_SAFE_MOVE_BOX_IN = .129;
    public static double ARM_SAFE_MOVE_BOX_OUT = .354;
    public static double TURRET_STRAIGHT = .518;
    public static double TURRET_LEFT_90 = .907;
    public static double TURRET_RIGHT_90 = .115; //2 * TURRET_STRAIGHT - TURRET_LEFT_90;
    // box positions are of boxPos, not boxServoPos
    public static double BOX_DEPLOY = .325;
    public static double BOX_INTAKE = .82;
    public static double BOX_HOLD = .6;
    public static double BOX_WITHDRAW_RELPOS = .413;
    public static double BOX_TAKE_OUT = .445;
    public static double BOX_CAP = .43;

    public static double BOX_HOLD_SHARED = .508;
    public static double TURRET_DEPLOY_RIGHT = .055;
    public static double TURRET_DEPLOY_LEFT = 1.0;

    public static double LINKAGE_SAFE_AMOUNT = 0.75;
    public static long LINKAGE_SAFE_TIME = 500;

    public static double LINKAGE_SHARED = .52;

    public static double armRate = .032;
    public static double boxAllianceRate = .025;
    public static double boxSharedRate = .025;
    public static double boxIntakeRate = .030;

    public static long ROTATE_TIME = 350;

    //TODO: min and max positions to avoid going too far
    public static double SLIDES_MAX = 1145;
    public static double SLIDES_MIN = 0;

    public static double LINKAGE_1_MAX_OUT = 0.09;
    public static double LINKAGE_1_MAX_IN = .67;
    public static double LINKAGE_2_MAX_OUT = 0.82;
    public static double LINKAGE_2_MAX_IN = .11;

    //TODO: adjust to driver preference
    public static long FAST_COOL_DOWN = 25;
    public static long NORMAL_COOL_DOWN = 100;

    public static int TOP = 820;
    public static int MID = 400;
    public static int CAP = 1150;

    //TODO: fix these adjustments as necessary
    public static double LINKAGE_ADJUSTMENT = .058;
    public static int SLIDES_ADJUSTMENT = 7;

    public static double INTAKE_POWER = 0.6;

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
    public static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 330);
    public static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(430, 330);
    public static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(860, 330);


    public static final int REGION_WIDTH = 100;
    public static final int REGION_HEIGHT = 100;
    //TODO: fix all the box detecting constants
}