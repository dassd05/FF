package org.firstinspires.ftc.teamcode.drive.testing;

import org.firstinspires.ftc.teamcode.util.Encoder;

import static org.firstinspires.ftc.teamcode.drive.Constants.*;

@Deprecated
public class LocalizationTesting {

    public Encoder leftEncoder, rightEncoder;

    //trying to implement own localization using wheel encoders
    //need an accurate way to measure ticks (and therefore velocity), but because resolution maxes out at 28 * whatever gear ratio we're using, accuracy becomes kinda difficult

    //so normally, you usually use velocity and multiply it by time to get change in position for differential drives, but because REV's getVelocity is garbage, using distance is probably better, though still not as accurate (again cause low ticks per revolution)

    double xPos, yPos, thetaPos;

    public void updatePos(double lastX, double lastY, double lastTheta, double deltaLeft,
                          double deltaRight, long updateTime) {

        double leftVelo = deltaLeft/updateTime; //(ticks/ms)
        double rightVelo = deltaRight/updateTime; //(ticks/ms)

        //calculate position for infinite radius (straight line)
        //calculations are super easy here since no change in angle
        if (deltaLeft == deltaRight) {
            thetaPos = lastTheta;
            xPos = lastX + (deltaLeft * Math.cos(lastTheta)); //integrates x position
            yPos = lastY + (deltaLeft * Math.sin(lastTheta)); //integrates y position
        } else {
            //now the hard part... calculating the change when the robot moves relative to an arc
            double radius = (trackWidth/2.0) * ((leftVelo + rightVelo)/(rightVelo - leftVelo)); //takes the overall velocity and divides it by the difference in velocity. Intuitive way to think of it is if my robot is moving super fast, then a small discrepancy in left and right wheel would barely make the robot curve (larger radius); however, if both of my wheels were moving super slowly, then even a slight difference in velocity would make it curve more (smaller radius)
            //don't have to worry about dividing by 0 as that case is only when deltaLeft==deltaRight, which is dealt with above

            double xCurvatureCenter = lastX - radius * Math.sin(lastTheta);
            double yCurvatureCenter = lastY - radius * Math.cos(lastTheta);
            //i think the calculations here might get thrown off if loops are too slow
            //now that we have the radius, we just take the the x and y lengths and subtract them from our current x and y position

            double deltaTheta = (deltaRight - deltaLeft) / trackWidth;
            //increases turning left

            thetaPos = lastTheta + deltaTheta;
            xPos = Math.cos(deltaTheta) * (lastX - xCurvatureCenter) - Math.sin(deltaTheta) * (lastY - yCurvatureCenter) + xCurvatureCenter;
            yPos = Math.sin(deltaTheta) * (lastX - xCurvatureCenter) - Math.cos(deltaTheta) * (lastY - yCurvatureCenter) + yCurvatureCenter;
            //got this part from the
        }
    }
}
