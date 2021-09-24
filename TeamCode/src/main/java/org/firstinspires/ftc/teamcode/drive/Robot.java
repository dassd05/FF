package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class Robot {
    public Servo someServo = null;

    public DcMotor Left1 = null, Left2 = null, Right1 = null, Right2 = null;

    public VoltageSensor batteryVoltageSensor = null;

    public BNO055IMU imu = null;

    HardwareMap hwMap = null;
    public Telemetry telemetry = null;

    Orientation angles = null;
    Acceleration gravity = null;

    public FtcDashboard dashboard = null;

    public Robot() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        Left1 = hwMap.get(DcMotor.class, "Left1");
        Left2 = hwMap.get(DcMotor.class, "Left2");
        Right1 = hwMap.get(DcMotor.class, "Right1");
        Right2 = hwMap.get(DcMotor.class, "Right2");

        Left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Right1.setDirection(DcMotorSimple.Direction.REVERSE);
        Right2.setDirection(DcMotorSimple.Direction.REVERSE);

        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        someServo = hwMap.get(Servo.class, "someServo");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.update();
        telemetry.clearAll();
    }

    public enum deployState {
        DOWN,
        MIDDLE,
        TOP,
        SHARED,
    }

    public deployState deploymentState;

    public ElapsedTime deployTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public boolean resetTimer = true;
    public boolean isPositionReached = false;

    public void updateDeployState() {
        switch (deploymentState) {
            case DOWN:
                break;

            case MIDDLE:
                break;

            case TOP:
                break;

            case SHARED:
                break;

            default:
        }
    }


    public void deployUp() {
        deploymentState = deployState.DOWN;
        deployTimer.reset();
    }
    public void deployMiddle() {
        deploymentState = deployState.MIDDLE;
        deployTimer.reset();
    }
    public void deployTop() {
        deploymentState = deployState.TOP;
        deployTimer.reset();
    }
    public void deployShared()  {
        deploymentState = deployState.SHARED;
        deployTimer.reset();
    }

    public void updateAllStates() {
        updateDeployState();
    }


    public ElapsedTime autonWaitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public enum LeftBlue {

    }
    public static LeftBlue LeftSideBlue;


    public enum MiddleBlue {

    }
    public static MiddleBlue MiddleSideBlue;


    public enum RightBlue {

    }
    public static RightBlue RightSideBlue;


    public enum LeftRed {

    }
    public static LeftRed LeftSideRed;


    public enum MiddleRed {

    }
    public static MiddleRed MiddleSideRed;


    public enum RightRed {

    }
    public static RightRed RightSideRed;

    public void composeTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });
    }

    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void setTankPowers(double left, double right) {
        Left1.setPower(left);
        Left2.setPower(left);
        Right1.setPower(right);
        Right2.setPower(right); //I think, don't know how they're geared together
    }
}
