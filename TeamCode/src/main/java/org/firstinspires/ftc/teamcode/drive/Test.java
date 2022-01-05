package org.firstinspires.ftc.teamcode.drive;

@Deprecated
public class Test {}

/*
package org.firstinspires.ftc.teamcode.HardwareClasses;


import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.PID;
import org.firstinspires.ftc.teamcode.utilities.RingBufferOwen;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.floorMod;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.PowerShot.PS_CLOSE;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.PowerShot.PS_FAR;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.PowerShot.PS_MID;
import static org.firstinspires.ftc.teamcode.utilities.MathUtils.pow;
import static org.firstinspires.ftc.teamcode.utilities.Utils.hardwareMap;

public class Robot {

    public static DcMotor frontLeft, frontRight, backLeft, backRight;

    private static final PID telePID = new PID(.02, 0, .004, 8);
    private static final PID autoPID = new PID(.02, 0, .003, 8);
    private static final PID visionPID = new PID(0.06, 0, .007, 10);

    static RingBufferOwen timeRing = new RingBufferOwen(3);
    static RingBufferOwen frRing = new RingBufferOwen(3);
    static RingBufferOwen flRing = new RingBufferOwen(3);
    static RingBufferOwen brRing = new RingBufferOwen(3);
    static RingBufferOwen blRing = new RingBufferOwen(3);

    private static final double TICKS_PER_ROTATION = 537.7;

    public static DriveState currentDriveState;

    public static double drive = 0, strafe = 0, turn = 0, power = 1;
    public static double targetAngle;
    public static double releaseAngle = 0;
    public static double adjRateOfChange = 0;

    private static double startAngle = 0;
    public static double currentInches = 0;
    public static boolean isStrafeComplete = true, isTurnComplete = true;

    private static int powerShotState = 3;
    private static double adjustmentAngle = 0;
    private static double towerAimAngle = 0;
    private static double PSAngle = 0;

    private static double maxRPM;


    public static void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentDriveState = DriveState.FULL_CONTROL;
    }


    public static void resetGyro(double offsetAngle){
        Sensors.gyro.update();
        Sensors.gyro.setDatum(Sensors.gyro.IMUAngle() + offsetAngle);
        Sensors.gyro.update();
        targetAngle = Sensors.gyro.rawAngle();
        releaseAngle = Sensors.gyro.rawAngle();
        adjRateOfChange = 0;
    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    public static double closestTarget(double targetAngle){
        double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + Sensors.gyro.rawAngle()) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
        return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? Sensors.gyro.rawAngle() - simpleTargetDelta : Sensors.gyro.rawAngle() - alternateTargetDelta;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static double absClosestTarget(double targetAngle){
        if(Sensors.alliance == Sensors.Alliance.RED) return closestTarget(targetAngle + 180);
        else return closestTarget(targetAngle);
    }


    //SET POWER METHODS


    public static void setPower(double drive, double strafe, double turn, double power){
        Robot.drive = Range.clip(drive, -1, 1);
        Robot.strafe = Range.clip(strafe, -1, 1);
        Robot.turn = Range.clip(turn, -1, 1);
        Robot.power = Range.clip(power, 0.05 , 1);

        double flPower = (Robot.drive - Robot.strafe - Robot.turn) * Robot.power;
        double frPower = (Robot.drive + Robot.strafe + Robot.turn) * Robot.power;
        double blPower = (Robot.drive + Robot.strafe - Robot.turn) * Robot.power;
        double brPower = (Robot.drive - Robot.strafe + Robot.turn) * Robot.power;

        double maxPower = abs(max(max(abs(flPower), abs(frPower)), max(abs(blPower), abs(brPower))));
        if(maxPower > 1) { frPower /= maxPower; flPower /= maxPower; blPower /= maxPower; brPower /= maxPower; }
        else if(maxPower < .05 && maxPower > -.05) { flPower = 0; frPower = 0; blPower = 0; brPower = 0; }

        setMotorPower(flPower, frPower, blPower, brPower);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void setPowerTele(double drive, double strafe, double turn, double power){
        double inputTurn;
        runWithoutEncoders();

        if(turn!= 0) {
            inputTurn = turn;
            releaseAngle = Sensors.gyro.rawAngle();
            adjRateOfChange = pow(Sensors.gyro.rateOfChange(), 2);
        }else if(adjRateOfChange > 1000){
            releaseAngle = Sensors.gyro.rawAngle();
            adjRateOfChange = pow(Sensors.gyro.rateOfChange(), 2);
            inputTurn = 0;
        }else{
            targetAngle = releaseAngle + .5 * .0035 * adjRateOfChange;
            inputTurn = telePID.update(closestTarget(targetAngle) - Sensors.gyro.rawAngle());
        }
        setPower(drive, strafe, inputTurn, power);
    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void setPowerAuto(double drive, double strafe, double targetAngle, double power){
        Robot.targetAngle = targetAngle;
        releaseAngle = Sensors.gyro.rawAngle();
        turn = autoPID.update(Robot.targetAngle - Sensors.gyro.rawAngle());
        setPower(drive, strafe, turn, power);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void setPowerAuto(double drive, double strafe, double targetAngle){
        setPowerAuto(drive, strafe, targetAngle,1.0);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void setPowerVision(double drive, double strafe, double targetAngle, double power){
        Robot.targetAngle = targetAngle;
        releaseAngle = Sensors.gyro.rawAngle();
        runWithEncoders();
        double error = targetAngle - Sensors.gyro.rawAngle();
        turn = visionPID.update(pow(error, .7));
        setPower(drive, strafe, turn, power);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void setPowerVision(double drive, double strafe, double targetAngle){
        setPowerVision(drive, strafe, targetAngle,1.0);
    }


    //AUTO METHODS


    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void strafe(double distance, double heading, double strafeAngle, double targetPower, double startPower, double endPower){

        distance = abs((distance) / 0.0207);
        startPower = abs(startPower);
        targetPower = abs(targetPower);
        endPower = abs(endPower);

        double deceleratePower;
        double accelRate = 0.0013;

        runWithoutEncoders();

        if (isStrafeComplete){
            resetWithoutEncoders();
        }

        double currentAngle = Sensors.gyro.rawAngle();

        double drive = Math.cos((strafeAngle - currentAngle) * PI / 180);
        double strafe = Math.sin((strafeAngle - currentAngle) * PI / 180);

        double currentDistance = adjustedTicks();

        if(distance != 0){
            double remainingDistance = distance - currentDistance;
            deceleratePower = Math.sqrt(accelRate * (remainingDistance + 1.0/accelRate * Math.pow(endPower, 2))) + .05;
            isStrafeComplete = currentDistance >= distance;
        }else{
            deceleratePower = 1;
            isStrafeComplete = false;
        }

        double acceleratePower = Math.sqrt(accelRate * (currentDistance + 1.0/accelRate * Math.pow(startPower, 2))) + .05;
        double currentPower = min(min(acceleratePower, deceleratePower), targetPower);

        currentInches = (0.0207 * currentDistance);

        setPowerAuto(drive, strafe, closestTarget(heading), currentPower);

    }



    public static double adjustedTicks() {
        double measuredTicks = ((abs(frontRight.getCurrentPosition()) + abs(frontLeft.getCurrentPosition()) + abs(backRight.getCurrentPosition()) + abs(backLeft.getCurrentPosition())) / 4.0);
        double angleAdjustment = (abs(abs(drive) - abs(strafe)) - 1) * -1;
        return Math.sqrt(Math.pow(measuredTicks, 2) + Math.pow(measuredTicks * angleAdjustment, 2));
    }



    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void turn(double targetAngle, double targetPower, double startPower){
        targetPower = abs(targetPower);
        double accelRate = 0.03;
        double currentAngle = Sensors.gyro.rawAngle();

        runWithEncoders();

        if(isTurnComplete){
            startAngle = Sensors.gyro.rawAngle();
            resetWithEncoders();
        }

        double acceleratePower = Math.sqrt(accelRate * (abs(currentAngle - startAngle)+ 1/accelRate * Math.pow(startPower, 2))) + .05;
        double currentPower = min(acceleratePower, targetPower);

        if(startAngle > closestTarget(targetAngle)){
            isTurnComplete = currentAngle < closestTarget(targetAngle) + 2.5;
        }else{
            isTurnComplete = currentAngle > closestTarget(targetAngle) - 2.5;
        }

        isStrafeComplete = isTurnComplete;

        setPowerAuto(0,0, closestTarget(targetAngle), currentPower);
    }


    //TELEOP METHODS


    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void driveState(double drive, double strafe, double turn, double power){
        switch ( currentDriveState){

            case FULL_CONTROL:
                setPowerTele(drive, strafe, turn, power);
                break;

            case NORTH:
                if(abs(turn) > 0.05) { newState(DriveState.FULL_CONTROL); break; }
                setPowerAuto(drive, strafe, closestTarget(0), power);
                break;

            case EAST:
                if(abs(turn) > 0.05) { newState(DriveState.FULL_CONTROL); break; }
                setPowerAuto(drive, strafe, closestTarget(270), power);
                break;

            case SOUTH:
                if(abs(turn) > 0.05) { newState(DriveState.FULL_CONTROL); break; }
                setPowerAuto(drive, strafe, closestTarget(180), power);
                break;

            case WEST:
                if(abs(turn) > 0.05) { newState(DriveState.FULL_CONTROL); break; }
                setPowerAuto(drive, strafe, closestTarget(90), power);
                break;

            case ADJUSTMENT:
                if(abs(turn) > 0.05) { newState(DriveState.FULL_CONTROL); break; }
                setPowerAuto(drive, strafe, getAdjustmentAngle(), power);
                break;

            case HIGH_GOAL_AIM:
                if(abs(turn) > 0.05) { newState(DriveState.FULL_CONTROL); break; }
                if (Sensors.frontCamera.isHighGoalFound() && Sensors.gyro.absAngleRange(35, 145)) { setPowerVision(drive, strafe, getTowerAimAngle() + Sensors.robotVelocityComponent(getTowerAimAngle() - 90) / 29, power); }
                else { setPowerAuto(drive, strafe, absClosestTarget(90)); }
                break;

            case MID_GOAL_AIM:
                if(abs(turn) > 0.05) { newState(DriveState.FULL_CONTROL); break; }
                if (Sensors.frontCamera.isMidGoalFound() && Sensors.gyro.absAngleRange(35, 145)) { setPowerVision(drive, strafe, getTowerAimAngle(), power); }
                else { setPowerAuto(drive, strafe, absClosestTarget(90)); }
                break;

            case POWER_SHOT_AIM:
                if(abs(turn) > 0.05) { newState(DriveState.FULL_CONTROL); break; }
                if (Sensors.frontCamera.isHighGoalFound() && Sensors.gyro.absAngleRange(20, 160)) { setPowerVision(drive, strafe, closestTarget(getPSAngle()), power); }
                else setPowerAuto(drive, strafe, absClosestTarget(90));
                break;
        }
    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    //if the boolean is true, it sets the targetAngle for the PID system to the closest coterminal angle to the input's respective angle (North 0, East 270, South 180, West 90) with priority being in that order
    public static void cardinalState(boolean north, boolean east, boolean south, boolean west){
        if(north) newState(DriveState.NORTH);
        if(east) newState(DriveState.EAST);
        if(west) newState(DriveState.WEST);
        if(south) newState(DriveState.SOUTH);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void autoAimState(boolean autoAim, boolean autoAimToggle, boolean adjRight, boolean adjLeft, double adjDeltaAngle){
        if(autoAimToggle && (Shooter.currentShooterState == Shooter.ShooterState.HIGH_GOAL)){
            if(autoAim || Shooter.shooterJustOn) { newState(DriveState.HIGH_GOAL_AIM); runWithEncoders(); }
            setTowerAimAngle(Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError());
        }

        if(autoAimToggle && (Shooter.currentShooterState == Shooter.ShooterState.MID_GOAL)){
            if(autoAim || Shooter.shooterJustOn) { newState(DriveState.MID_GOAL_AIM); runWithEncoders(); }
            setTowerAimAngle(Sensors.gyro.rawAngle() + Sensors.frontCamera.midGoalError());
        }

        if(autoAimToggle && (Shooter.currentShooterState == Shooter.ShooterState.OFF) && Shooter.shooterJustOn) {
            targetAngle = Sensors.gyro.rawAngle();
            releaseAngle = Sensors.gyro.rawAngle();
            newState(DriveState.FULL_CONTROL);
        }


        if(Shooter.currentShooterState == Shooter.ShooterState.POWER_SHOT) {
            if (autoAimToggle && (autoAim || Shooter.shooterJustOn)) newState(DriveState.POWER_SHOT_AIM);

            if (Shooter.shooterJustOn) powerShotState = 1;
            if (Shooter.feederJustOn || adjRight) powerShotState++;
            if (adjLeft) powerShotState--;
            powerShotState %= 3;

            switch (powerShotState) {
                case 0:
                    setPSAngle(Sensors.frontCamera.getPowerShotAngle(PS_CLOSE));
                    break;
                case 1:
                    setPSAngle(Sensors.frontCamera.getPowerShotAngle(PS_MID));
                    break;
                case 2:
                    setPSAngle(Sensors.frontCamera.getPowerShotAngle(PS_FAR));
                    break;
            }
        }else {
            if (adjRight) {
                newState(DriveState.ADJUSTMENT);
                setAdjustmentAngle(Sensors.gyro.rawAngle() - adjDeltaAngle);
            }
            if (adjLeft) {
                newState(DriveState.ADJUSTMENT);
                setAdjustmentAngle(Sensors.gyro.rawAngle() + adjDeltaAngle);
            }
        }
    }

    public static void setAdjustmentAngle(double adjustmentAngle){ Robot.adjustmentAngle = adjustmentAngle; }

    public static double getAdjustmentAngle(){ return adjustmentAngle; }

    public static void setTowerAimAngle(double towerAimAngle){ Robot.towerAimAngle = towerAimAngle; }

    public static double getTowerAimAngle(){ return towerAimAngle; }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void setPSAngle(double PSAngle){ Robot.PSAngle = PSAngle - 1.5; }

    public static double getPSAngle(){ return PSAngle; }


    //MOTOR METHODS


    public static void setMotorPower(double flPower, double frPower, double blPower, double brPower){
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    public static double maxRPM() {
        long currentTime = System.currentTimeMillis();
        long deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaMinutes = deltaMili / 60000.0;

        long frPosition = frontRight.getCurrentPosition();
        long frDeltaTicks = abs(frPosition - frRing.getValue(frPosition));

        long flPosition = frontLeft.getCurrentPosition();
        long flDeltaTicks = abs(flPosition - flRing.getValue(flPosition));

        long brPosition = backRight.getCurrentPosition();
        long brDeltaTicks = abs(brPosition - brRing.getValue(brPosition));

        long blPosition = backLeft.getCurrentPosition();
        long blDeltaTicks = abs(blPosition - blRing.getValue(blPosition));

        long maxDeltaTicks = max(max(frDeltaTicks, flDeltaTicks), max(blDeltaTicks, brDeltaTicks));


        double maxDeltaRotations = maxDeltaTicks / TICKS_PER_ROTATION;

        return Math.abs(maxDeltaRotations / deltaMinutes);
    }

    public static void runWithEncoders(){
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void runWithoutEncoders(){
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void stopAndResetEncodes(){
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static void resetWithoutEncoders(){
        stopAndResetEncodes();
        runWithoutEncoders();
    }

    public static void resetWithEncoders(){
        stopAndResetEncodes();
        runWithEncoders();
    }


    //STATE MACHINE STUFF


    private static void newState(DriveState newState) {
        currentDriveState = newState;
        runWithoutEncoders();
    }

    private enum DriveState {
        FULL_CONTROL, NORTH, EAST, SOUTH, WEST, ADJUSTMENT, HIGH_GOAL_AIM, MID_GOAL_AIM, POWER_SHOT_AIM
    }

}

**/
