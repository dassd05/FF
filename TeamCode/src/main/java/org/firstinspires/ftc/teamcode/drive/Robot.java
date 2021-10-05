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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.*;

import static org.firstinspires.ftc.teamcode.drive.Autons.Vision.BoxPositionDetection.pipeline;
import static org.firstinspires.ftc.teamcode.drive.Constants.Constants.*;


public class Robot {
    public Servo someServo = null;

    public DcMotor Left1 = null, Left2 = null, Right1 = null, Right2 = null;

    public VoltageSensor batteryVoltageSensor = null;

    public BNO055IMU imu = null;

    public Encoder leftEncoder = null, rightEncoder = null;

    public WebcamName webcamName;
    public OpenCvWebcam webcam;

    HardwareMap hwMap = null;
    public Telemetry telemetry = null;

    Orientation angles = null;
    Acceleration gravity = null;

    public FtcDashboard dashboard = null;

    public Robot() {

    }

    public void init(HardwareMap ahwMap) {
        //TODO: hardware mappings
        hwMap = ahwMap;

        Left1 = hwMap.get(DcMotorEx.class, "Left1");
        Left2 = hwMap.get(DcMotor.class, "Left2");
        Right1 = hwMap.get(DcMotor.class, "Right1");
        Right2 = hwMap.get(DcMotor.class, "Right2");

        Left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Right1.setDirection(DcMotorSimple.Direction.REVERSE);
        Right2.setDirection(DcMotorSimple.Direction.REVERSE);

        rightEncoder = new Encoder(hwMap.get(DcMotorEx.class, "something"));
        leftEncoder = new Encoder(hwMap.get(DcMotorEx.class, "something"));

        // TODO: reverse any encoders if needed
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        someServo = hwMap.get(Servo.class, "someServo");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        List<DcMotor> motors = Arrays.asList(Left1, Left2, Right1, Right2); //etc.etc.

        for (DcMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.update();
        telemetry.clearAll();
    }

    public void webcamInit(HardwareMap ahwMap) {
        hwMap = ahwMap;

        int cameraMonitorViewId = hwMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());

        webcamName = hwMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
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
    public static LeftBlue LeftBlueState;


    public enum MiddleBlue {

    }
    public static MiddleBlue MiddleBlueState;


    public enum RightBlue {

    }
    public static RightBlue RightBlueState;

    public enum LeftBlue2 {

    }
    public static LeftBlue2 LeftBlueState2;


    public enum MiddleBlue2 {

    }
    public static MiddleBlue2 MiddleBlueState2;


    public enum RightBlue2 {

    }
    public static RightBlue2 RightBlueState2;


    public enum LeftRed {

    }
    public static LeftRed LeftRedState;


    public enum MiddleRed {

    }
    public static MiddleRed MiddleRedState;


    public enum RightRed {

    }
    public static RightRed RightRedState;


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
        Right2.setPower(right);
    }

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    double lastError = 0;
    double integral = 0;
    double error = 0;

    public void PIDDrive (double target, double left, double right, boolean runPID) {
        if (runPID) {
            PIDTimer.reset();
            double currentHeading = getAngle();
            error = target - currentHeading;
            double deltaError = error - lastError;
            integral += error * PIDTimer.time();
            double derivative = deltaError / PIDTimer.time();

            double P = pidConsts.p * error;
            double I = pidConsts.i * integral;
            double D = pidConsts.d * derivative;

            double PID = P + I + D;

            setTankPowers(left + PID, right - PID); //I think

            lastError = error;
        } else {
            lastError = 0;
        }
    }

    //absolutely no idea how you do this without like using rr with its quintic splines or whatever
    public void runToPoint () {

    }

    //need to spend time to make this so that slippage is minimized
    //tbh, we'll probably do a ramp up where you Range.clip the velocity up till a certain point
    public void accelerate() {

    }

    public void decelerate() {

    }


    public ElapsedTime odoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public double getLeftWheelVelo() {
        return encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * LEFT_WHEEL_MULTIPLIER;
    }
    public double getLeftWheelPos() {
        return encoderTicksToInches(leftEncoder.getCurrentPosition()) * LEFT_WHEEL_MULTIPLIER;
    }

    public double getRightWheelVelo() {
        return encoderTicksToInches(rightEncoder.getCorrectedVelocity() * RIGHT_WHEEL_MULTIPLIER);
    }
    public double getRightWheelPos() {
        return encoderTicksToInches(rightEncoder.getCurrentPosition()) * RIGHT_WHEEL_MULTIPLIER;
    }


    public static double xPos, yPos, thetaPos;

    //new update method that utilizes velocity instead of positions
    //might need to make adjustments (e.g. using position change rather than getVelo, using position instead of velo * time, etc.)
    public void updatePos(double lastX, double lastY, double lastTheta, double update /* in millisecond*/) {

        getRightWheelPos();
        getLeftWheelPos();
        double leftVelo = getLeftWheelVelo();
        double rightVelo = getRightWheelVelo();

        //calculate position for infinite radius (straight line)
        //calculations are super easy here since no change in angle
        if (leftVelo == rightVelo) {
            thetaPos = lastTheta;
            xPos = lastX + leftVelo * (update / 1000.0) * Math.cos(lastTheta); //integrates x position
            yPos = lastY + rightVelo * (update / 1000.0) * Math.sin(lastTheta); //integrates y position
        } else {
            //now the hard part... calculating the change when the robot moves relative to an arc
            double radius = (trackWidth/2.0) * ((leftVelo + rightVelo)/(rightVelo - leftVelo)); //takes the overall velocity and divides it by the difference in velocity. Intuitive way to think of it is if my robot is moving super fast, then a small discrepancy in left and right wheel would barely make the robot curve (larger radius); however, if both of my wheels were moving super slowly, then even a slight difference in velocity would make it curve more (smaller radius)
            //don't have to worry about dividing by 0 as that case is only when deltaLeft==deltaRight, which is dealt with above

            double xCurvatureCenter = lastX - radius * Math.sin(lastTheta);
            double yCurvatureCenter = lastY + radius * Math.cos(lastTheta);
            //i think the calculations here might get thrown off if loops are too slow
            //now that we have the radius, we just take the the x and y lengths and subtract them from our current x and y position

            double deltaTheta = ((getRightWheelVelo() - getLeftWheelVelo()) / trackWidth) * (update * 1000.0);
            //increases turning left

            thetaPos = lastTheta + deltaTheta;
            xPos = Math.cos(deltaTheta) * (lastX - xCurvatureCenter) - Math.sin(deltaTheta) * (lastY - yCurvatureCenter) + xCurvatureCenter;
            yPos = Math.sin(deltaTheta) * (lastX - xCurvatureCenter) - Math.cos(deltaTheta) * (lastY - yCurvatureCenter) + yCurvatureCenter;
            odoTimer.reset();
        }
    }

    public double getX() {
        return xPos;
    }
    public double getY() {
        return yPos;
    }
    public double getTheta() {
        return thetaPos;
    }
}
