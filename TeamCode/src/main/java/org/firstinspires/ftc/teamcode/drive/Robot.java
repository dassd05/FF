package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.*;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.Constants.*;


/**
 * Robot is a class which contains all the functions of the robot.
 *
 * <p>To use this class, first call the constructor with your {@link HardwareMap} and
 * {@link Telemetry}. During the initialization phase, call the desired init functions of Robot.
 * You may thereafter call the functions of the Robot as desired.</p>
 */
@SuppressWarnings("unused")
public class Robot {

    //----------------------------------------------------------------------------------------------
    // Robot State
    //----------------------------------------------------------------------------------------------

    public static LeftBlue LeftBlueState;
    public static MiddleBlue MiddleBlueState;
    public static RightBlue RightBlueState;
    public static LeftBlue2 LeftBlueState2;
    public static MiddleBlue2 MiddleBlueState2;
    public static RightBlue2 RightBlueState2;
    public static LeftRed LeftRedState;
    public static MiddleRed MiddleRedState;
    public static RightRed RightRedState;

    //----------------------------------------------------------------------------------------------
    // Parameters
    //----------------------------------------------------------------------------------------------

    /**
     * position variables for the robot
     */
    public double xPos, yPos, thetaPos;

    public DcMotor frontLeft, backLeft, frontRight, backRight;
    public DcMotor intake;
    public DcMotorEx slides1, slides2;
    public Servo boxServo, linkage1, linkage2;
    public CRServoImplEx carousel1, carousel2;
    public VoltageSensor voltageSensor;
    public BNO055IMU imu;
    public Encoder leftEncoder, rightEncoder;
    public WebcamName webcamName;
    public OpenCvWebcam webcam;

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public FtcDashboard dashboard;

    public DeployState deploymentState;
    public DropState dropState;
    public ElapsedTime safeDropTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public int desiredSlidesPosition = 0;
    public double slidesPower = 0.0;
    public IntakeState intakeState;
    public BoxState boxState;
    public double linkageAdjustment = 0.0;
    public int slidesAdjustment = 0;
    public ElapsedTime slidesTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double errorSlides1 = 0.0;
    //    public ElapsedTime autonWaitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime odoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double errorSlides2 = 0.0;

    //public ElapsedTime boxTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    //dont need it
    double lastErrorSlides1 = 0.0;
    double lastErrorSlides2 = 0.0;
    double integralSlides1 = 0.0;
    double integralSlides2 = 0.0;

    //    public enum TeamShippingElementState {
//
//    }
    ElapsedTime PIDDriveTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double lastErrorDrive = 0.0;
    double integralDrive = 0.0;
    double errorDrive = 0.0;

    //----------------------------------------------------------------------------------------------
    // Initialization
    //----------------------------------------------------------------------------------------------

    /**
     * Constructor for the {@link Robot} class. <i>Does not do the initialization!</i>
     *
     * @param hardwareMap the hardwareMap you get from {@link com.qualcomm.robotcore.eventloop.opmode.OpMode}
     * @param telemetry   the telemetry you get from {@link com.qualcomm.robotcore.eventloop.opmode.OpMode}
     * @see #init()
     * @see #webcamInit(OpenCvPipeline)
     * @see #dashboardInit()
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    /**
     * Initialize most of the hardware for the robot.
     *
     * @see #webcamInit(OpenCvPipeline)
     * @see #dashboardInit()
     */
    public void init() {
        //TODO: hardware mappings
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE); //this is correct

        slides1 = hardwareMap.get(DcMotorEx.class, "slides1");
        slides2 = hardwareMap.get(DcMotorEx.class, "slides2");

        slides1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slides1.setTargetPosition(0);
        slides1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slides2.setTargetPosition(0);
        slides2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE); //this too

//        rightEncoder = new Encoder(hwMap.get(DcMotorEx.class, "something"));
//        leftEncoder = new Encoder(hwMap.get(DcMotorEx.class, "something"));
//
//        // TODO: reverse any encoders if needed
//        leftEncoder.setDirection(Encoder.Direction.REVERSE);
//        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        boxServo = hardwareMap.get(Servo.class, "boxServo");
        carousel1 = hardwareMap.get(CRServoImplEx.class, "carousel1");
        carousel2 = hardwareMap.get(CRServoImplEx.class, "carousel2");
        linkage1 = hardwareMap.get(Servo.class, "linkage1");
        linkage2 = hardwareMap.get(Servo.class, "linkage2");

        //noinspection DuplicatedCode
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
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
        deploymentState = DeployState.REST;
        boxState = BoxState.COLLECT;
        dropState = DropState.DROP;

        telemetry.update();
        telemetry.clearAll();
    }

    /**
     * Initializes the {@link OpenCvWebcam} for the robot.
     *
     * @param pipeline the pipeline to set for the webcam
     * @see #init()
     * @see #dashboardInit()
     */
    public void webcamInit(OpenCvPipeline pipeline) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
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

    /**
     * Initializes an {@link FtcDashboard} for the robot, as well as the telemetry for the dashboard.
     *
     * @see #init()
     * @see #webcamInit(OpenCvPipeline)
     */
    public void dashboardInit() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    //----------------------------------------------------------------------------------------------
    // Updating State
    //----------------------------------------------------------------------------------------------

    /**
     * Put the deployment in the resting state.
     *
     * @see #deployMiddle()
     * @see #deployTop()
     * @see #deployShared()
     */
    public void deployRest() {
        deploymentState = DeployState.REST;
        dropState = DropState.DROP;
    }

    /**
     * Move the deployment into the position for the middle rack of the alliance shipping hub.
     *
     * @see #deployRest()
     * @see #deployTop()
     * @see #deployShared()
     */
    public void deployMiddle() {
        deploymentState = DeployState.MIDDLE;
    }

    /**
     * Move the deployment into the position for the top rack of the alliance shipping hub.
     *
     * @see #deployRest()
     * @see #deployMiddle()
     * @see #deployShared()
     */
    public void deployTop() {
        deploymentState = DeployState.TOP;
    }

    /**
     * Move the deployment into the position for the shared shipping hub.
     *
     * @see #deployRest()
     * @see #deployMiddle()
     * @see #deployTop()
     */
    public void deployShared() {
        deploymentState = DeployState.SHARED;
    }

    /**
     * Turn the intake on.
     *
     * @see #intakeOff()
     * @see #intakeReverse()
     */
    public void intakeOn() {
        intakeState = IntakeState.ON;
    }

    /**
     * Turn the intake off.
     *
     * @see #intakeOn()
     * @see #intakeReverse()
     */
    public void intakeOff() {
        intakeState = IntakeState.OFF;
    }

    /**
     * Make the intake rotate in the reverse direction
     *
     * @see #intakeOn()
     * @see #intakeOff()
     */
    public void intakeReverse() {
        intakeState = IntakeState.REVERSE;
    }

    /**
     * Rotate the box into the drop-off state.
     *
     * @see #liftBox()
     * @see #collectBox()
     */
    public void dropoffBox() {
        boxState = BoxState.DROP;
    }

    /**
     * Rotate the box into an upwards state.
     *
     * @see #dropoffBox()
     * @see #collectBox()
     */
    public void liftBox() {
        boxState = BoxState.UP;
    }

    /**
     * Rotate the box into position for intaking.
     *
     * @see #dropoffBox()
     * @see #liftBox()
     */
    public void collectBox() {
        boxState = BoxState.COLLECT;
    }


    //todo clean this up
    public ElapsedTime safeDropTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime deployTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public int desiredSlidesPosition = 0;
//    public double power = 0.0;
    public double position = 0.0;

    boolean firstTime = false;

    /**
     * Update the deployment FSM.
     *
     * @see #updateIntakeState()
     * @see #updateBoxState()
     * @see #updateAllStates()
     */
    public void updateDeployState() {
        switch (deploymentState) {
            case REST:
                if (firstTime) {
                    resetSlidesAdjustment();
                    resetLinkageAdjustment();
                }

                position = 0;

                collectBox();

                switch (dropState) {
                    case DROP:
                        if (deployTimer.time() > ROTATE_TIME) {
                            if (linkage2.getPosition() < LINKAGE_SAFE_DROP) {
                                if (slides1.getCurrentPosition() > 35) {
                                    desiredSlidesPosition = 35;
                                    slidesPower = .85;
                                } else {
                                    safeDropTimer.reset();
                                    dropState = DropState.FINAL;
                                }
                            }
                        }
                        break;

                    case FINAL:
                        if (safeDropTimer.time() > 250) {
                            desiredSlidesPosition = 0;
                            slidesPower = .3;
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
                    slidesPower = .8;

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
                    slidesPower = .85;

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
                    slidesPower = .8;

                    if (getSlides1CurrentPosition() > LINKAGE_SAFE_EXTEND)
                        position = .4;
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

    public void moveSlides(int targetPosition, double power) {
        slides1.setTargetPosition(targetPosition);
        slides1.setPower(power);

        slides2.setTargetPosition(targetPosition);
        slides2.setPower(power);
    }

    public void moveLinkage(double targetPosition) {
        linkage2.setPosition(targetPosition);
        linkage1.setPosition(1-targetPosition);
    }

    /**
     * Update the intake FSM.
     *
     * @see #updateDeployState()
     * @see #updateBoxState()
     * @see #updateAllStates()
     */
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

    /**
     * Update the box FSM.
     *
     * @see #updateDeployState()
     * @see #updateIntakeState()
     * @see #updateAllStates()
     */
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
        }
    }

    /**
     * Update all FSMs on the robot.
     *
     * @see #updateDeployState()
     * @see #updateIntakeState()
     * @see #updateBoxState()
     */
    public void updateAllStates() {
        updateDeployState();
        updateIntakeState();
        updateBoxState();
    }

    /**
     * Moves the slides to a target position at a particular power.
     *
     * @param targetPosition the desired encoder target position for the slides motor
     * @param power          the power to set the motors to
     */
    public void moveSlides(int targetPosition, double power) {
        slides1.setTargetPosition(targetPosition);
        slides1.setPower(power);

        slides2.setTargetPosition(targetPosition);
        slides2.setPower(power);
    }

    /**
     * Adjust the linkage by a particular amount.
     *
     * @param adjust
     * @see #resetLinkageAdjustment()
     */
    public void linkageAdjust(double adjust) {
        linkageAdjustment += adjust;
    }

    /**
     * Reset the linkage's adjustment.
     *
     * @see #linkageAdjust(double)
     */
    public void resetLinkageAdjustment() {
        linkageAdjustment = 0.0;
    }

    /**
     * Adjust the slides by a particular amount.
     *
     * @param adjust
     * @see #resetSlidesAdjustment()
     */
    public void slidesAdjust(int adjust) {
        slidesAdjustment += adjust;
    }

    /**
     * Reset the slides' adjustment.
     *
     * @see #slidesAdjust(int)
     */
    public void resetSlidesAdjustment() {
        slidesAdjustment = 0;
    }

    public double getSlides1CurrentPosition() {
        return slides1.getCurrentPosition();
    }

    public double getSlides2CurrentPosition() {
        return slides2.getCurrentPosition();
    }

    //rip not using
    @Deprecated
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

    /**
     * Get the heading of the robot, according to imu.
     *
     * @return the heading of the robot, in radians
     */
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //might want to experiment with extrinsic, but Im not sure if it will actually reduce drift
    }

    public double getOdoAngle() {
        if (getThetaDegrees() > 180)
            return (360 - getThetaDegrees());
        else
            return -getThetaDegrees(); //we switch the signs cause odo returns increasing angle counter clockwise
    }

    /**
     * Set motor powers for a tank drive, where you can control the speed of the left and right
     * side of the drivetrain.
     *
     * @param left  number in the range [-1, 1] representing the velocity to set the left side to
     * @param right number in the range [-1, 1] representing the velocity to set the right side to
     */
    public void setTankPowers(double left, double right) {
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }

    /**
     * Set motor powers for a tank drive, where you can control the forward velocity (relative to
     * the robot) and rotational speed. You can also add a multiplier to adjust the sensitivity.
     *
     * @param forward    number in the range [-1, 1] representing the relative forward velocity
     * @param turn       number in the range [-1, 1] representing the velocity to set the left side to,
     *                   with positive representing clockwise motion and negative representing
     *                   counterclockwise motion.
     * @param multiplier <i><b>PROBABLY SHOULD GET RID OF, SINCE WE CAN DO IT MANUALLY
     *                   OUTSIDE OF THE FUNCTION.</b></i>
     */
    public void setTankPowers(double forward, double turn, double multiplier) {
        frontLeft.setPower((forward + turn) * multiplier);
        backLeft.setPower((forward + turn) * multiplier);
        frontRight.setPower((forward - turn) * multiplier);
        backRight.setPower((forward - turn) * multiplier);
    }

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

    /**
     * Clear the bulk cache for each {@link LynxModule} in the robot.
     */
    public void clearCache() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }
    }

    /**
     * Converts ticks on the encoder to inches.
     *
     * @param ticks number of encoder ticks
     * @return converted distance in inches
     */
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public enum DeployState {
        REST,
        MIDDLE,
        TOP,
        SHARED,
    }

    public enum DropState {
        DROP,
        FINAL
    }

    public enum IntakeState {
        OFF,
        REVERSE,
        ON
    }

    public enum BoxState {
        COLLECT,
        UP,
        DROP
    }

    public enum LeftBlue {

    }

    public enum MiddleBlue {

    }


    public enum RightBlue {

    }

    public enum LeftBlue2 {

    }

    public enum MiddleBlue2 {

    }

    public enum RightBlue2 {

    }

    public enum LeftRed {

    }

    public enum MiddleRed {

    }

    public enum RightRed {

    }

}
