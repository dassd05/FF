package org.firstinspires.ftc.teamcode.drive.Autons.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.drive.Constants.*;

public class BoxPositionDetection {

    public static BoxDetection pipeline = new BoxDetection();

    public enum BoxPosition {
        RIGHT,
        MIDDLE,
        LEFT
    }

    public static class BoxDetection extends OpenCvPipeline {

        private final Scalar BLUE = new Scalar(0, 0, 255);
        private final Scalar GREEN = new Scalar(0, 255, 0);

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        public volatile BoxPosition position;

        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);

            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    BLUE,
                    2);

            Imgproc.rectangle(
                    input,
                    region3_pointA,
                    region3_pointB,
                    BLUE,
                    2);

            int greater1or2 = Math.max(avg1, avg2);
            int greatest = Math.max(greater1or2, avg3);

            if(greatest == avg1) {
                position = BoxPosition.LEFT;

                Imgproc.rectangle(
                        input,
                        region1_pointA,
                        region1_pointB,
                        GREEN,
                        -1);
            }
            else if(greatest == avg2) {
                position = BoxPosition.MIDDLE;

                Imgproc.rectangle(
                        input,
                        region2_pointA,
                        region2_pointB,
                        GREEN,
                        -1);
            }
            else if(greatest == avg3) {
                position = BoxPosition.RIGHT;

                Imgproc.rectangle(
                        input,
                        region3_pointA,
                        region3_pointB,
                        GREEN,
                        -1);
            }
            return input;
        }
    }
}