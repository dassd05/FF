package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
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
import org.firstinspires.ftc.teamcode.drive.Autons.Blue.Blue2;
import org.firstinspires.ftc.teamcode.drive.Autons.Vision.BoxPositionDetection;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.*;

import static org.firstinspires.ftc.teamcode.drive.Autons.Vision.BoxPositionDetection.pipeline;
import static org.firstinspires.ftc.teamcode.drive.Constants.*;


public class Robot {
    public Servo boxServo = null, linkage1 = null, linkage2 = null, capper = null;
    public CRServo carousel2;

    public DcMotor frontLeft = null, backLeft = null, frontRight = null, backRight = null;

    public DcMotor intake = null;

    public DcMotorEx slides1 = null, slides2 = null;

    public DcMotor carousel;

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

        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE); //this is correct

        slides1 = hwMap.get(DcMotorEx.class, "slides1");
        slides2 = hwMap.get(DcMotorEx.class, "slides2");

        slides1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slides1.setTargetPosition(0);
        slides1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slides2.setTargetPosition(0);
        slides2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake = hwMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE); //this too

        carousel = hwMap.get(DcMotor.class, "port3");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        rightEncoder = new Encoder(hwMap.get(DcMotorEx.class, "something"));
//        leftEncoder = new Encoder(hwMap.get(DcMotorEx.class, "something"));
//
//        // TODO: reverse any encoders if needed
//        leftEncoder.setDirection(Encoder.Direction.REVERSE);
//        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        boxServo = hwMap.get(Servo.class, "boxServo");
        //carousel1 = hwMap.get(CRServoImplEx.class, "carousel1");
        carousel2 = hwMap.get(CRServo.class, "carousel2");
        linkage1 = hwMap.get(Servo.class, "linkage1");
        linkage2 = hwMap.get(Servo.class, "linkage2");
        capper = hwMap.get(Servo.class, "capper");

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
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        List<DcMotor> motors = Arrays.asList(frontLeft, backLeft, frontRight, backRight, intake); //etc.etc.
        List<DcMotorEx> motorsEx = Arrays.asList(slides1, slides2);

        for (DcMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
        for (DcMotorEx motor : motorsEx) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        intakeState = IntakeState.OFF;
        deploymentState = deployState.REST;
        boxState = BoxState.COLLECT;
        dropState = drop.DROP;

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
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
        REST,
        MIDDLE,
        TOP,
        SHARED,
        CAP_HOVER,
        CAP_UP
    }

    public deployState deploymentState;

    public enum drop {
        DROP,
        FINAL
    }

    public drop dropState;

    public ElapsedTime safeDropTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime deployTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public int desiredSlidesPosition = 0;
    public double power = 0.0;
    public double position = 0.0;

    boolean firstTime = false;

    public void updateDeployState() {
        switch (deploymentState) {
            case REST:
                if (firstTime) {
                    resetSlidesAdjustment();
                    resetLinkageAdjustment();
                    collectBox();
                    firstTime = false;
                }

                position = 0;

                switch (dropState) {
                    case DROP:
                        if (deployTimer.time() > ROTATE_TIME) {
                            if (linkage2.getPosition() < LINKAGE_SAFE_DROP) {
                                if (slides1.getCurrentPosition() > 25 && deployTimer.time() < 4000 /*to make sure it moves on*/) {
                                    desiredSlidesPosition = 25;
                                    power = .85;
                                } else {
                                    safeDropTimer.reset();
                                    dropState = drop.FINAL;
                                }
                            }
                        }
                        break;

                    case FINAL:
                        if (safeDropTimer.time() > 200) {
                            desiredSlidesPosition = 0;
                            power = .3;
                        }
                        break;
                }

                break;

            case MIDDLE:
                if (firstTime) {
                    liftBox();
                    firstTime = false;
                }

                if (deployTimer.time() > ROTATE_TIME) {
                    desiredSlidesPosition = (int) Range.clip(MID + slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
                    power = .8;

                    if (getSlides1CurrentPosition() > LINKAGE_SAFE_EXTEND)
                        position = .4;
                }
                break;

            case TOP:
                if (firstTime) {
                    liftBox();
                    firstTime = false;
                }

                if (deployTimer.time() > ROTATE_TIME) {
                    desiredSlidesPosition = (int) Range.clip(TOP + slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
                    power = .85;

                    if (getSlides1CurrentPosition() > LINKAGE_SAFE_EXTEND)
                        position = .4;
                }
                break;

            case SHARED:
                if (firstTime) {
                    liftBox();
                    firstTime = false;
                }

                if (deployTimer.time() > ROTATE_TIME) {
                    desiredSlidesPosition = (int) Range.clip(LOW + slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
                    power = .8;

                    if (getSlides1CurrentPosition() > LINKAGE_SAFE_EXTEND)
                        position = .4;
                }
                break;

            case CAP_HOVER:
                if (firstTime) {
                    resetBoxAdjustment();
                    firstTime = false;
                }

                desiredSlidesPosition = (int) Range.clip(HOVER + slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
                power = .8;

                if (getSlides1CurrentPosition() > LINKAGE_SAFE_EXTEND) {
                    position = .2;
                    hoverBox();
                }
                break;

            case CAP_UP:
                desiredSlidesPosition = (int) Range.clip(CAP + slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
                power = .4;

                if (getSlides1CurrentPosition() > LINKAGE_SAFE_EXTEND) {
                    position = .2;
                    capBox();
                }
                break;

            default:
        }
    }


    public void deployRest() {
        deploymentState = deployState.REST;
        dropState = drop.DROP;
        firstTime = true;
        deployTimer.reset();
    }

    public void deployMiddle() {
        deploymentState = deployState.MIDDLE;
        deployTimer.reset();
        firstTime = true;
    }

    public void deployTop() {
        deploymentState = deployState.TOP;
        deployTimer.reset();
        firstTime = true;
    }

    public void deployShared() {
        deploymentState = deployState.SHARED;
        deployTimer.reset();
        firstTime = true;
    }

    public void hover() {
        deploymentState = deployState.CAP_HOVER;
        deployTimer.reset();
        firstTime = true;
    }

    public void cap() {
        deploymentState = deployState.CAP_UP;
        deployTimer.reset();
    }

    public void moveSlides(int targetPosition, double power) {
        slides1.setTargetPosition(targetPosition);
        slides1.setPower(power);

        slides2.setTargetPosition(targetPosition);
        slides2.setPower(power);
    }

    public void moveLinkage(double targetPosition) {
        // target position is from 0 to 1 max in to out
        linkage1.setPosition(Range.scale(targetPosition, 0, 1, LINKAGE_1_MAX_IN, LINKAGE_1_MAX_OUT));
        linkage2.setPosition(Range.scale(targetPosition, 0, 1, LINKAGE_2_MAX_IN, LINKAGE_2_MAX_OUT));
    }

    public enum IntakeState {
        OFF,
        REVERSE,
        ON
    }

    public IntakeState intakeState;

    public void updateIntakeState() {
        switch (intakeState) {
            case ON:
                intake.setPower(INTAKE_POWER);
                break;

            case REVERSE:
                intake.setPower(-INTAKE_POWER);
                break;

            case OFF:
                intake.setPower(0.0);
                break;
        }
    }

    public void intakeOn() {
        intakeState = IntakeState.ON;
    }

    public void intakeOff() {
        intakeState = IntakeState.OFF;
    }

    public void intakeReverse() {
        intakeState = IntakeState.REVERSE;
    }

    public enum BoxState {
        COLLECT,
        UP,
        DROP,
        HOVER,
        FINAL_CAP
    }

    public BoxState boxState;

    //public ElapsedTime boxTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    //dont need it

    public void updateBoxState() {
        switch (boxState) {
            case DROP:
                boxServo.setPosition(BOX_ROTATION_DEPLOY);
                break;
            case UP:
                boxServo.setPosition(BOX_ROTATION_UP);
                break;
            case COLLECT:
                boxServo.setPosition(BOX_ROTATION_DOWN);
                break;
            case HOVER:
                boxServo.setPosition(BOX_ROTATION_HOVER + boxAdjustment);
                break;
            case FINAL_CAP:
                boxServo.setPosition(BOX_ROTATION_CAP + boxAdjustment);
                break;
        }
    }

    public double boxAdjustment = 0.0;

    public void boxAdjust(double adjust) {
        boxAdjustment += adjust;
    }

    public void resetBoxAdjustment() {
        boxAdjustment = 0.0;
    }


    public void dropoffBox() {
        boxState = BoxState.DROP;
    }

    public void liftBox() {
        boxState = BoxState.UP;
    }

    public void collectBox() {
        boxState = BoxState.COLLECT;
    }

    public void hoverBox() {
        boxState = BoxState.HOVER;
    }
    public void capBox() {
        boxState = BoxState.FINAL_CAP;
    }

//    public enum TeamShippingElementState {
//
//    }

    public void updateAllStates() {
        updateDeployState();
        updateIntakeState();
        updateBoxState();
    }

    public void updateAll() {
        updateAllStates();
        moveSlides(desiredSlidesPosition, power);
        moveLinkage(Range.clip(position + linkageAdjustment, 0, 1));
    }

    public double linkageAdjustment = 0.0;

    public void linkageAdjust(double adjust) {
        linkageAdjustment += adjust;
    }

    public void resetLinkageAdjustment() {
        linkageAdjustment = 0.0;
    }

    public int slidesAdjustment = 0;

    public void slidesAdjust(int adjust) {
        slidesAdjustment += adjust;
    }

    public void resetSlidesAdjustment() {
        slidesAdjustment = 0;
    }


    public double getSlides1CurrentPosition() {
        return slides1.getCurrentPosition();
    }

    public double getSlides2CurrentPosition() {
        return slides2.getCurrentPosition();
    }

    public ElapsedTime slidesTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public double errorSlides1 = 0.0;
    double errorSlides2 = 0.0;
    double lastErrorSlides1 = 0.0;
    double lastErrorSlides2 = 0.0;
    double integralSlides1 = 0.0;
    double integralSlides2 = 0.0;

    //rip not using
    public void linearSlidesPID(double position, double update, boolean runPID) {
        if (runPID) {
            errorSlides1 = position - getSlides1CurrentPosition();
            errorSlides2 = position - getSlides2CurrentPosition();
            double deltaError1 = errorSlides1 - lastErrorSlides1;
            double deltaError2 = errorSlides2 - lastErrorSlides2;
            integralSlides1 += errorSlides1 * update;
            integralSlides2 += errorSlides2 * update;
            double derivative1 = deltaError1 / update;
            double derivative2 = deltaError2 / update;
            slidesTimer.reset();

            double P1 = pidConstsSlides.p * errorSlides1;
            double P2 = pidConstsSlides.p * errorSlides2;
            double I1 = pidConstsSlides.i * integralSlides1;
            double I2 = pidConstsSlides.i * integralSlides2;
            double D1 = pidConstsSlides.d * derivative1;
            double D2 = pidConstsSlides.d * derivative2;

            slides1.setPower(P1 + I1 + D1);
            slides2.setPower(P2 + I2 + D2);

            lastErrorSlides1 = errorSlides1;
            lastErrorSlides2 = errorSlides2;
        } else {
            double errorSlides1 = 0.0;
            double errorSlides2 = 0.0;
            double lastErrorSlides1 = 0.0;
            double lastErrorSlides2 = 0.0;
            double integralSlides1 = 0.0;
            double integralSlides2 = 0.0;
            slides1.setPower(0);
            slides2.setPower(0);
            slidesTimer.reset();
        }
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
        FORWARD,
        TURN,
        EXTEND,
        DROP,
        FORWARD2,
        TURN2,
        PARK,
        FINISH,
        CAROUSEL,
        ACTUAL_FINISH
    }

    public static LeftRed LeftRedState;


    public enum MiddleRed {
        FORWARD,
        TURN,
        EXTEND,
        DROP,
        FORWARD2,
        TURN2,
        PARK,
        FINISH,
        CAROUSEL,
        ACTUAL_FINISH
    }

    public static MiddleRed MiddleRedState;


    public enum RightRed {
        FORWARD,
        TURN,
        EXTEND,
        DROP,
        FORWARD2,
        TURN2,
        PARK,
        FINISH,
        CAROUSEL,
        ACTUAL_FINISH
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
        return -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //might want to experiment with extrinsic, but Im not sure if it will actually reduce drift
    }

    public double getOdoAngle() {
        if (getThetaDegrees() > 180)
            return (360 - getThetaDegrees());
        else
            return -getThetaDegrees(); //we switch the signs cause odo returns increasing angle counter clockwise
    }

    public void setTankPowers(double left, double right) {
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }

    public void setTankPowers(double forward, double turn, double multiplier) {
        frontLeft.setPower((forward + turn) * multiplier);
        backLeft.setPower((forward + turn) * multiplier);
        frontRight.setPower((forward - turn) * multiplier);
        backRight.setPower((forward - turn) * multiplier);
    }

    public void gyroStraight(double power, double heading) {
        double error = (heading - getAngle()) * .025;
        setTankPowers(power + error, power - error);
    }

    public void resetWheels() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    ElapsedTime PIDDriveTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    double lastErrorDrive = 0.0;
    double integralDrive = 0.0;
    double errorDrive = 0.0;

    public void PIDDrive(double target, double left, double right, boolean runPID, long update) {
        if (runPID) {
            //probably maybe switch to getOdoAngle() cause no I2C lag plus this imu seems to drift
            //also kinda need a new hub, this one isn't very good at all
            double currentHeading = getAngle();
            errorDrive = target - currentHeading;
            double deltaError = errorDrive - lastErrorDrive;
            integralDrive += errorDrive * PIDDriveTimer.time();
            double derivative = deltaError / update;
            PIDDriveTimer.reset();

            double P = pidConsts.p * errorDrive;
            double I = pidConsts.i * integralDrive;
            double D = pidConsts.d * derivative;

            double PID = P + I + D;

            setTankPowers(left + PID, right - PID); //I think

            lastErrorDrive = errorDrive;
        } else {
            lastErrorDrive = 0.0;
            integralDrive = 0.0;
            errorDrive = 0.0;
            PIDDriveTimer.reset();
        }
    }

    //absolutely no idea how you do this without like using rr with its quintic splines or whatever
    public void runToPoint() {

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

    public double backLeftPosition() {
        return backLeft.getCurrentPosition();
    }
    public double frontLeftPosition() {
        return frontLeft.getCurrentPosition();
    }
    public double backRightPosition() {
        return intake.getCurrentPosition();
    }
    public double frontRightPosition() {
        return -carousel.getCurrentPosition();
    }


    public static double xPos, yPos, thetaPos;

    //new update method that utilizes velocity instead of positions
    //might need to make adjustments (e.g. using position change rather than getVelo, using position instead of velo * time, etc.)
    public void updatePos(double lastX, double lastY, double lastTheta, double update /* in millisecond*/ /*,
                          boolean imuUpdate  idea that sets a timer and once it gets to a setpoint
                          (i.e. every 500 ms), it will update the angle to prevent significant drift over
                          time while not having to go through a slow I2C call every update*/) {

        getRightWheelPos();
        getLeftWheelPos();
        double leftVelo = getLeftWheelVelo();
        double rightVelo = getRightWheelVelo();

        //calculate position for infinite radius (straight line)
        //calculations are super easy here since no change in angle
        if (leftVelo == rightVelo) {
            // if(imuUpdate)
            //      lastTheta = getAngle(); update method to prevent angle drift over time
            // else
            //      thetaPos = lastTheta;
            thetaPos = lastTheta;
            xPos = lastX + leftVelo * (update / 1000.0) * Math.cos(lastTheta); //integrates x position
            yPos = lastY + rightVelo * (update / 1000.0) * Math.sin(lastTheta); //integrates y position
            odoTimer.reset();
        } else {
            //now the hard part... calculating the change when the robot moves relative to an arc
            double radius = (trackWidth / 2.0) * ((leftVelo + rightVelo) / (rightVelo - leftVelo)); //takes the overall velocity and divides it by the difference in velocity. Intuitive way to think of it is if my robot is moving super fast, then a small discrepancy in left and right wheel would barely make the robot curve (larger radius); however, if both of my wheels were moving super slowly, then even a slight difference in velocity would make it curve more (smaller radius)
            //don't have to worry about dividing by 0 as that case is only when deltaLeft==deltaRight, which is dealt with above

            double xCurvatureCenter = lastX - radius * Math.sin(lastTheta);
            double yCurvatureCenter = lastY + radius * Math.cos(lastTheta);
            //i think the calculations here might get thrown off if loops are too slow
            //now that we have the radius, we just take the the x and y lengths and subtract them from our current x and y position

            double deltaTheta = ((getRightWheelVelo() - getLeftWheelVelo()) / trackWidth) * (update * 1000.0);
            //increases turning left
            odoTimer.reset();

            // if (imuUpdate)
            //      thetaPos = getAngle();
            // else {
            //      if (Math.toDegrees((lastTheta + deltaTheta)) > 360)
            //          thetaPos = (lastTheta + deltaTheta) - Math.toRadians(360);
            //      else
            //          thetaPos = (lastTheta + deltaTheta);
            //      }
            //
            // alternate method to update angle through imu every now and then (i.e. every 500 ms) to reduce cycle time

            //thetaPos = lastTheta + deltaTheta;

            // added checks to maintain angle between 0 - 360
            if (Math.toDegrees((lastTheta + deltaTheta)) > 360)
                thetaPos = (lastTheta + deltaTheta) - Math.toRadians(360);
            else if (Math.toDegrees(lastTheta + deltaTheta) < 0)
                thetaPos = (lastTheta + deltaTheta) + Math.toRadians(360);
            else
                thetaPos = (lastTheta + deltaTheta);

            xPos = Math.cos(deltaTheta) * (lastX - xCurvatureCenter) - Math.sin(deltaTheta) * (lastY - yCurvatureCenter) + xCurvatureCenter;
            yPos = Math.sin(deltaTheta) * (lastX - xCurvatureCenter) - Math.cos(deltaTheta) * (lastY - yCurvatureCenter) + yCurvatureCenter;
            //TODO: test out odo localization
            // if it doesn't work, we need to get started with rr unfortunately
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
    public double getThetaDegrees() {
        return Math.toDegrees(thetaPos);
    }

    public void clearCache() {
        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }
    }
}