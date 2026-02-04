package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name="FindStartPositions")
public class FindStartPositions extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 800;
    double fy = 800;
    double cx = 320;
    double cy = 240;

    double tagSize = 0.2065;

    int[] aprilTagIDs = {20, 24};

    double startX = 0;
    double startY = 0;
    double startHeading = 0;

    @Override
    public void runOpMode() {

        int camViewId = hardwareMap.appContext.getResources()
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance()
            .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT); }
            @Override public void onError(int error) {}
        });

        waitForStart();

        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

        if (detections.size() != 0) {
            for (AprilTagDetection tag : detections) {
                for (int id : aprilTagIDs) {
                    if (tag.id == id) {
                        startX = tag.pose.x;
                        startY = tag.pose.y;
                        startHeading = Math.toDegrees(tag.pose.yaw);
                        telemetry.addData("Detected Tag", tag.id);
                        telemetry.addData("Pos X", startX);
                        telemetry.addData("Pos Y", startY);
                        telemetry.addData("Heading", startHeading);
                        telemetry.update();
                        break;
                    }
                }
            }
        } else {
            telemetry.addLine("No valid AprilTag found");
            telemetry.update();
        }

        telemetry.addLine("Autonomous ready");
        telemetry.update();
    }
}
