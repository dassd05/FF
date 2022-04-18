package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
    public Servo boxServo = null, linkage1 = null, linkage2 = null,
    turret = null, arm = null;
    public CRServo carousel2;

    public DcMotor frontLeft = null, backLeft = null, frontRight = null, backRight = null;

    public DcMotor intake = null;

    public DcMotorEx slides1 = null, slides2 = null;

    public DcMotor carousel;

    public RevColorSensorV3 boxSensor = null;

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

        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        turret = hwMap.get(Servo.class, "turret");
        arm = hwMap.get(Servo.class, "arm");

        boxSensor = hwMap.get(RevColorSensorV3.class, "boxSensor");

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
        boxState = BoxState.INTAKE;
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
        SHARED_LEFT,
        SHARED_RIGHT,
        CAP_HOVER,
        CAP_UP,
        BOTTOM
    }

    public deployState deploymentState;

    public enum drop {
        DROP,
        FINAL
    }

    public drop dropState;

    public ElapsedTime safeDropTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime deployTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public ElapsedTime armTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public int desiredSlidesPosition = 0;
    public double power = 0.0;
    public double position = 0.0;

    public double turretPos = 0.0;
    public double armPos = 0.0;
    public double boxServoPos = 0.0;

    public double boxAbsPos = BOX_HOLD;
    public double boxRelPos;

    boolean firstTime = false;
    boolean slowDown = false;

    boolean absPos = true;

    public boolean freightLoaded = false;
    public boolean intakeOn = false;
    public boolean reverse = false;
    public boolean manualBoxUp = false;

    //old deployment w/o turret go brrrrrrrr
//    public void updateDeployState() {
//        switch (deploymentState) {
//            case REST:
//                if (firstTime) {
//                    resetSlidesAdjustment();
//                    resetLinkageAdjustment();
//                    collectBox();
//                    firstTime = false;
//                }
//
//                position = 0;
//
//                switch (dropState) {
//                    case DROP:
//                        if (deployTimer.time() > ROTATE_TIME) {
//                            if (linkage2.getPosition() < LINKAGE_SAFE_DROP) {
//                                if (slides1.getCurrentPosition() > 25 && deployTimer.time() < 4000 /*to make sure it moves on*/) {
//                                    desiredSlidesPosition = 25;
//                                    power = .85;
//                                } else {
//                                    safeDropTimer.reset();
//                                    dropState = drop.FINAL;
//                                }
//                            }
//                        }
//                        break;
//
//                    case FINAL:
//                        if (safeDropTimer.time() > 200) {
//                            desiredSlidesPosition = 0;
//                            power = .3;
//                        }
//                        break;
//                }
//
//                break;
//
//            case MIDDLE:
//                if (firstTime) {
//                    liftBox();
//                    firstTime = false;
//                }
//
//                if (deployTimer.time() > ROTATE_TIME) {
//                    desiredSlidesPosition = (int) Range.clip(MID + slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
//                    power = .8;
//
//                    if (getSlides1CurrentPosition() > LINKAGE_SAFE_EXTEND)
//                        position = .4;
//                }
//                break;
//
//            case TOP:
//                if (firstTime) {
//                    liftBox();
//                    firstTime = false;
//                }
//
//                if (deployTimer.time() > ROTATE_TIME) {
//                    desiredSlidesPosition = (int) Range.clip(TOP + slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
//                    power = .85;
//
//                    if (getSlides1CurrentPosition() > LINKAGE_SAFE_EXTEND)
//                        position = .4;
//                }
//                break;
//
//            case SHARED:
//                if (firstTime) {
//                    liftBox();
//                    firstTime = false;
//                }
//
//                if (deployTimer.time() > ROTATE_TIME) {
//                    desiredSlidesPosition = (int) Range.clip(LOW + slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
//                    power = .8;
//
//                    if (getSlides1CurrentPosition() > LINKAGE_SAFE_EXTEND)
//                        position = .4;
//                }
//                break;
//
//            case CAP_HOVER:
//                if (firstTime) {
//                    resetBoxAdjustment();
//                    firstTime = false;
//                }
//
//                desiredSlidesPosition = (int) Range.clip(HOVER + slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
//                power = .8;
//
//                if (getSlides1CurrentPosition() > LINKAGE_SAFE_EXTEND) {
//                    position = .2;
//                    hoverBox();
//                }
//                break;
//
//            case CAP_UP:
//                desiredSlidesPosition = (int) Range.clip(CAP + slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
//                power = .4;
//
//                if (getSlides1CurrentPosition() > LINKAGE_SAFE_EXTEND) {
//                    position = .2;
//                    capBox();
//                }
//                break;
//
//            default:
//        }
//    }

    public void updateDeployState() {
        switch (deploymentState) {
            case REST:
                if (firstTime) {
                    if (Math.abs(turret.getPosition() - TURRET_STRAIGHT) > .1) {
                        slowDown = true;
                    } else {
                        slowDown = false;
                    }
                    resetSlidesAdjustment();
                    if (!slowDown)
                        resetLinkageAdjustment();
                    resetTurretAdjustment();
                    resetArmAdjustment();
                    resetBoxAdjustment();
                    firstTime = false;
                }
                if (slowDown) {
                    turretPos = TURRET_STRAIGHT;
                    bringIn();
                    armPos = ARM_VERTICAL;

                    if (deployTimer.time() > 750) {
                        slowDown = false;
                    }
                } else {
                    resetLinkageAdjustment();
                    turretPos = TURRET_STRAIGHT;
                    position = 0;

                    if (armPos < ARM_MAX_IN && Math.abs(ARM_MAX_IN - armPos) > armRate + .003) {
                        armPos += armRate;
                    } else if (armPos > ARM_MAX_IN && Math.abs(ARM_MAX_IN - armPos) > armRate + .003) {
                        armPos -= armRate;
                    } else {
                        armPos = ARM_MAX_IN;
                    }


                    if (!freightLoaded) {
                        if (arm.getPosition() <= ARM_SAFE_MOVE_BOX_IN) {
                            if (manualBoxUp)
                                boxUp();
                            else
                                intakeBox();
                        } else {
                            bringIn();
                        }

                        if (intakeOn) {
                            if (boxState == BoxState.INTAKE) {
                                if (reverse)
                                    intakeReverse();
                                else
                                    intakeOn();
                            }
                        } else {
                            intakeOff();
                        }
                    } else {
                        boxUp();

                        if (intakeOn) {
                            intakeReverse();
                        } else {
                            intakeOff();
                        }
                    }
                }

                switch (dropState) {
                    case DROP:
                        if (deployTimer.time() > ROTATE_TIME) {
                            if (slides1.getCurrentPosition() > 25 && deployTimer.time() < 4000 /*to make sure it moves on*/) {
                                desiredSlidesPosition = 25;
                                power = .7;
                            } else {
                                safeDropTimer.reset();
                                dropState = drop.FINAL;
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
                if (armPos >= ARM_VERTICAL) {
                    position = 0;
                } else {
                    position = LINKAGE_SAFE_AMOUNT;
                }

                turretPos = TURRET_STRAIGHT;

                if (deployTimer.time() > LINKAGE_SAFE_TIME) {
                    if (slides1.getCurrentPosition() < (MID + slidesAdjustment) - 200) {
                        if (armPos < ARM_VERTICAL && Math.abs(ARM_VERTICAL - armPos) > armRate + .003) {
                            armPos += armRate;
                        } else if (armPos > ARM_VERTICAL && Math.abs(ARM_VERTICAL - armPos) > armRate + .003) {
                            armPos -= armRate;
                        } else {
                            armPos = ARM_VERTICAL;
                        }
                    } else {
                        if (armPos < ARM_STRAIGHT_OUT && Math.abs(ARM_STRAIGHT_OUT - armPos) > armRate + .003) {
                            armPos += armRate;
                        } else if (armPos > ARM_STRAIGHT_OUT && Math.abs(ARM_STRAIGHT_OUT - armPos) > armRate + .003) {
                            armPos -= armRate;
                        } else {
                            armPos = ARM_STRAIGHT_OUT;
                        }
                    }
                }

                if (position == 0) {
                    desiredSlidesPosition = (int) Range.clip(MID + slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
                    power = .7;
                }

                if (boxState != BoxState.DEPLOY_ALLIANCE && boxState != BoxState.DEPLOY_SHARED ) {
                        boxUp();
                }

                break;

            case TOP:
                if (armPos >= ARM_VERTICAL) {
                    position = 0;
                } else {
                    position = LINKAGE_SAFE_AMOUNT;
                }

                turretPos = TURRET_STRAIGHT;

                if (deployTimer.time() > LINKAGE_SAFE_TIME) {
                    if (slides1.getCurrentPosition() < (TOP + slidesAdjustment) - 600) {
                        if (armPos < ARM_VERTICAL && Math.abs(ARM_VERTICAL - armPos) > armRate + .003) {
                            armPos += armRate;
                        } else if (armPos > ARM_VERTICAL && Math.abs(ARM_VERTICAL - armPos) > armRate + .003) {
                            armPos -= armRate;
                        } else {
                            armPos = ARM_VERTICAL;
                        }
                    } else {
                        if (armPos < ARM_STRAIGHT_OUT && Math.abs(ARM_STRAIGHT_OUT - armPos) > armRate + .003) {
                            armPos += armRate;
                        } else if (armPos > ARM_STRAIGHT_OUT && Math.abs(ARM_STRAIGHT_OUT - armPos) > armRate + .003) {
                            armPos -= armRate;
                        } else {
                            armPos = ARM_STRAIGHT_OUT;
                        }
                    }
                }

                if (position == 0) {
                    desiredSlidesPosition = (int) Range.clip(TOP + slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
                    power = .7;
                }

                if (boxState != BoxState.DEPLOY_ALLIANCE && boxState != BoxState.DEPLOY_SHARED ) {
                    boxUp();
                }
                break;

            case BOTTOM:

                if (armPos >= ARM_VERTICAL) {
                    position = 0;
                } else {
                    position = LINKAGE_SAFE_AMOUNT;
                }

                turretPos = TURRET_STRAIGHT;

                if (deployTimer.time() > LINKAGE_SAFE_TIME) {
                    if (armPos < ARM_STRAIGHT_OUT && Math.abs(ARM_STRAIGHT_OUT - armPos) > armRate + .003) {
                        armPos += armRate;
                    } else if (armPos > ARM_STRAIGHT_OUT && Math.abs(ARM_STRAIGHT_OUT - armPos) > armRate + .003) {
                        armPos -= armRate;
                    } else {
                        armPos = ARM_STRAIGHT_OUT;
                    }
                }

                if (boxState != BoxState.DEPLOY_ALLIANCE && boxState != BoxState.DEPLOY_SHARED ) {
                    boxUp();
                }

                break;

            case SHARED_LEFT:
                if (turretPos == TURRET_DEPLOY_RIGHT)
                    position = LINKAGE_SHARED;
                else
                    position = LINKAGE_SAFE_AMOUNT;

                if (deployTimer.time() > LINKAGE_SAFE_TIME) {
                    if (deployTimer.time() < LINKAGE_SAFE_TIME + 750) {
                        if (armPos < ARM_VERTICAL && Math.abs(ARM_VERTICAL - armPos) > armRate + .003) {
                            armPos += armRate;
                        } else if (armPos > ARM_VERTICAL && Math.abs(ARM_VERTICAL - armPos) > armRate + .003) {
                            armPos -= armRate;
                        } else {
                            armPos = ARM_VERTICAL;
                        }
                    } else {
                        turretPos = TURRET_DEPLOY_RIGHT;

                        if (armPos < ARM_STRAIGHT_OUT && Math.abs(ARM_STRAIGHT_OUT - armPos) > armRate + .003) {
                            armPos += armRate;
                        } else if (armPos > ARM_STRAIGHT_OUT && Math.abs(ARM_STRAIGHT_OUT - armPos) > armRate + .003) {
                            armPos -= armRate;
                        } else {
                            armPos = ARM_STRAIGHT_OUT;
                        }
                    }

                    if (boxState != BoxState.DEPLOY_ALLIANCE && boxState != BoxState.DEPLOY_SHARED ) {
                        boxUp();
                    }
                }

                desiredSlidesPosition = (int) Range.clip(slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
                power = .5;

                break;

            case SHARED_RIGHT:
                if (turretPos == TURRET_DEPLOY_LEFT)
                    position = LINKAGE_SHARED;
                else
                    position = LINKAGE_SAFE_AMOUNT;

                if (deployTimer.time() > LINKAGE_SAFE_TIME) {
                    if (deployTimer.time() < LINKAGE_SAFE_TIME + 750) {
                        if (armPos < ARM_VERTICAL && Math.abs(ARM_VERTICAL - armPos) > armRate + .003) {
                            armPos += armRate;
                        } else if (armPos > ARM_VERTICAL && Math.abs(ARM_VERTICAL - armPos) > armRate + .003) {
                            armPos -= armRate;
                        } else {
                            armPos = ARM_VERTICAL;
                        }
                    } else {
                        turretPos = TURRET_DEPLOY_LEFT;

                        if (armPos < ARM_STRAIGHT_OUT && Math.abs(ARM_STRAIGHT_OUT - armPos) > armRate + .003) {
                            armPos += armRate;
                        } else if (armPos > ARM_STRAIGHT_OUT && Math.abs(ARM_STRAIGHT_OUT - armPos) > armRate + .003) {
                            armPos -= armRate;
                        } else {
                            armPos = ARM_STRAIGHT_OUT;
                        }
                    }

                    if (boxState != BoxState.DEPLOY_ALLIANCE && boxState != BoxState.DEPLOY_SHARED ) {
                        boxUp();
                    }
                }

                desiredSlidesPosition = (int) Range.clip(slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
                power = .5;

                break;

            case CAP_UP:
                if (firstTime) {
                    resetSlidesAdjustment();
                    resetArmAdjustment();
                    firstTime = false;
                }
                position = LINKAGE_SHARED;

                if (armPos < ARM_CAP && Math.abs(ARM_CAP - armPos) > armRate + .003) {
                    armPos += armRate;
                } else if (armPos > ARM_CAP && Math.abs(ARM_CAP - armPos) > armRate + .003) {
                    armPos -= armRate;
                } else {
                    armPos = ARM_CAP;
                }

                desiredSlidesPosition = (int) Range.clip(CAP + slidesAdjustment, SLIDES_MIN, SLIDES_MAX);
                power = .7;

                break;

            case CAP_HOVER:
                if (arm.getPosition() >= ARM_VERTICAL)
                    position = 0;
                else
                    position = LINKAGE_SAFE_AMOUNT;

                turretPos = TURRET_STRAIGHT;

                if (deployTimer.time() > LINKAGE_SAFE_TIME) {
                    if (armPos < ARM_STRAIGHT_OUT && Math.abs(ARM_STRAIGHT_OUT - armPos) > armRate + .003) {
                        armPos += armRate;
                    } else if (armPos > ARM_STRAIGHT_OUT && Math.abs(ARM_STRAIGHT_OUT - armPos) > armRate + .003) {
                        armPos -= armRate;
                    } else {
                        armPos = ARM_STRAIGHT_OUT;
                    }

                    if (boxState != BoxState.DEPLOY_ALLIANCE && boxState != BoxState.DEPLOY_SHARED ) {
                        if (arm.getPosition() >= ARM_VERTICAL) {
                            capBox();
                        } else {
                            boxUp();
                        }
                    }
                }
                break;
            default:
                break;
        }
    }


    public void deployRest() {
        deploymentState = deployState.REST;
        dropState = drop.DROP;
        firstTime = true;
        deployTimer.reset();
        armTimer.reset();
    }

    public void deployMiddle() {
        deploymentState = deployState.MIDDLE;
        deployTimer.reset();
        firstTime = true;
    }

    public void deployBottom() {
        deploymentState = deployState.BOTTOM;
        deployTimer.reset();
        firstTime = true;
    }

    public void deployTop() {
        deploymentState = deployState.TOP;
        deployTimer.reset();
        firstTime = true;
    }

    public void deploySharedLeft() {
        deploymentState = deployState.SHARED_RIGHT;
        deployTimer.reset();
        firstTime = true;
    }

    public void deploySharedRight() {
        deploymentState = deployState.SHARED_LEFT;
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

    public double turretAdjustment = 0.0;

    public void turretAdjust(double adjust) {
            turretAdjustment += adjust;
    }

    public void resetTurretAdjustment() {
        turretAdjustment = 0.0;
    }

    public double armAdjustment = 0.0;

    public void armAdjust(double adjust) {
        armAdjustment += adjust;
    }

    public void resetArmAdjustment() {
        armAdjustment = 0.0;
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
        INTAKE,
        DEPLOY_SHARED,
        DEPLOY_ALLIANCE,
        BRING_IN,
        UP,
        UP_SHARED,
        TAKE_OUT,
        CAP
    }

    public BoxState boxState;

    boolean capworkplease = false;

    public void updateBoxState() {
        switch (boxState) {
            case INTAKE:
                absPos = true;
                if (boxAbsPos < BOX_INTAKE && Math.abs(BOX_INTAKE - boxAbsPos) > boxIntakeRate + .003) {
                    boxAbsPos += boxIntakeRate;
                } else if (boxAbsPos > BOX_INTAKE && Math.abs(BOX_INTAKE - boxAbsPos) > boxIntakeRate + .003) {
                    boxAbsPos -= boxIntakeRate;
                } else {
                    boxAbsPos = BOX_INTAKE;
                }
                break;
            case BRING_IN:
                absPos = false;
                boxRelPos = BOX_WITHDRAW_RELPOS;
                break;
            case TAKE_OUT:
                absPos = true;
                boxAbsPos = BOX_TAKE_OUT;
                break;
            case CAP:
                absPos = true;
                boxAbsPos = BOX_CAP;
                break;
            case UP:
                absPos = true;
                boxAbsPos = BOX_HOLD;
                break;
            case UP_SHARED:
                absPos = true;
                boxAbsPos = BOX_HOLD_SHARED;
                break;
            case DEPLOY_SHARED:
                absPos = true;
                if (boxAbsPos < BOX_DEPLOY && Math.abs(BOX_DEPLOY - boxAbsPos) > boxSharedRate + .003) {
                    boxAbsPos += boxSharedRate;
                } else if (boxAbsPos > BOX_DEPLOY && Math.abs(BOX_DEPLOY - boxAbsPos) > boxSharedRate + .003) {
                    boxAbsPos -= boxSharedRate;
                } else {
                    boxAbsPos = BOX_DEPLOY;
                }
                break;
            case DEPLOY_ALLIANCE:
                absPos = true;
                if (boxAbsPos < BOX_DEPLOY && Math.abs(BOX_DEPLOY - boxAbsPos) > boxAllianceRate + .003) {
                    boxAbsPos += boxAllianceRate;
                } else if (boxAbsPos > BOX_DEPLOY && Math.abs(BOX_DEPLOY - boxAbsPos) > boxAllianceRate + .003) {
                    boxAbsPos -= boxAllianceRate;
                } else {
                    boxAbsPos = BOX_DEPLOY;
                }
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

    public void intakeBox() {
        boxState = BoxState.INTAKE;
    }

    public void bringIn() {
        boxState = BoxState.BRING_IN;
    }

    public void takeOut() {
        boxState = BoxState.TAKE_OUT;
    }

    public void boxUp() {
        boxState = BoxState.UP;
    }

    public void upShared() {
        boxState = BoxState.UP_SHARED;
    }

    public void deployAlliance() {
        boxState = BoxState.DEPLOY_ALLIANCE;
    }

    public void deployShared() {
        boxState = BoxState.DEPLOY_SHARED;
    }

    public void capBox() {
        boxState = BoxState.CAP;
    }



    public void updateAllStates() {
        updateDeployState();
        updateIntakeState();
        updateBoxState();
    }

    public void updateAll() {
        updateAllStates();
        moveSlides(desiredSlidesPosition, power);
        moveLinkage(Range.clip(position + linkageAdjustment, 0, 1));

        if (absPos)
            boxServoPos = boxAbsPos - 4 * (armPos + armAdjustment) / 14;
        else
            boxServoPos = boxRelPos - 8 * (armPos + armAdjustment) / 14;

        turret.setPosition(turretPos + turretAdjustment);
        arm.setPosition(armPos + armAdjustment);
        boxServo.setPosition(boxServoPos);
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


    public ElapsedTime turnTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    double errorAngle = 0.0;
    double lastErrorAngle = 0.0;
    double integralErrorAngle = 0.0;
    double lastIntegralErrorAngle = 0.0;
    public boolean runPID = false;

    public double P = 0.0;
    public double I = 0.0;
    public double D = 0.0;

    public void turnPID(double position, double update, boolean runPID) {
        if (runPID) {
            errorAngle = getAngle() - position;
            double deltaError = errorAngle - lastErrorAngle;
            integralErrorAngle += errorAngle /** update*/;
            double derivative = deltaError / update;
            turnTimer.reset();

            if (errorAngle > 0 && lastErrorAngle < 0)
                integralErrorAngle = 0.0;
            else if (errorAngle < 0 && lastErrorAngle > 0)
                integralErrorAngle = 0.0;

            P = pidConstsTurn.p * errorAngle;
            I = pidConstsTurn.i * integralErrorAngle;
            D = pidConstsTurn.d * derivative;

            setTankPowers(-(P + I + D), (P + I + D));

            lastErrorSlides1 = errorSlides1;
            lastErrorSlides2 = errorSlides2;
            lastIntegralErrorAngle = integralErrorAngle;
        } else {
            errorAngle = 0.0;
            lastErrorAngle = 0.0;
            integralErrorAngle = 0.0;
            lastIntegralErrorAngle = 0.0;
            P = 0;
            I = 0;
            D = 0;
            setTankPowers(0,0);
            turnTimer.reset();
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
        BACK,
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