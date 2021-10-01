package org.firstinspires.ftc.teamcode.drive.Autons.Blue;

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

import static org.firstinspires.ftc.teamcode.drive.Constants.Constants.*;

public abstract class BoxPositionDetection extends LinearOpMode {

    protected WebcamName webcamName;
    protected OpenCvWebcam webcam;


    public void webcamInitialize() {
//        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        webcam.setPipeline(pipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
//            }
//        });
        //TODO: fix the webcam initialization
    }

    public int cameraMonitorViewId = hardwareMap.appContext.getResources().
            getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

    protected static BoxDetection pipeline = new BoxDetection();

    public enum BoxPosition {
        RIGHT,
        MIDDLE,
        LEFT
    }

    public static class BoxDetection extends OpenCvPipeline {

        private final Scalar BLUE = new Scalar(0, 0, 255);
        private final Scalar GREEN = new Scalar(0, 255, 0);

        Point region1_pointA = new Point(
                LEFT_REGION1_TOPLEFT_ANCHOR_POINT.x,
                LEFT_REGION1_TOPLEFT_ANCHOR_POINT.y
        );
        Point region1_pointB = new Point(
                LEFT_REGION1_TOPLEFT_ANCHOR_POINT.x + LEFT_REGION_WIDTH,
                LEFT_REGION1_TOPLEFT_ANCHOR_POINT.y + LEFT_REGION_HEIGHT
        );

        Point region2_pointA = new Point(
                LEFT_REGION2_TOPLEFT_ANCHOR_POINT.x,
                LEFT_REGION2_TOPLEFT_ANCHOR_POINT.y
        );
        Point region2_pointB = new Point(
                LEFT_REGION2_TOPLEFT_ANCHOR_POINT.x + LEFT_REGION_WIDTH,
                LEFT_REGION2_TOPLEFT_ANCHOR_POINT.y + LEFT_REGION_HEIGHT
        );

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        static int avg1;

        Mat region2_Cb;
        Mat YCrCb2 = new Mat();
        Mat Cb2 = new Mat();
        static int avg2;

        public volatile BoxPosition position;

        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }
        void inputToCb2(Mat input) {
            Imgproc.cvtColor(input, YCrCb2, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb2, Cb2, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);
            inputToCb2(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb2.submat(new Rect(region2_pointA, region2_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            inputToCb2(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];

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


            if (avg1 > THRESHOLD) {
                position = BoxPosition.LEFT;
            } else if (avg2 > THRESHOLD) {
                position = BoxPosition.MIDDLE;
            } else {
                position = BoxPosition.RIGHT;
            }

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    GREEN,
                    -1);
            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    GREEN,
                    -1);

            return input;
        }
    }
}
