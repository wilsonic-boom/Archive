package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "AprilTag Enhanced Localization")
public class AprilTagLinear extends LinearOpMode {

    private static final String WEBCAM_NAME = "Webcam 1";

    // --- CALIBRATION ---
    // Distance from the CENTER of the robot to the camera lens
    private static final double CAMERA_FORWARD_OFFSET = 6.0; // e.g., camera is 6" in front of center
    private static final double CAMERA_LEFT_OFFSET = 0.0;    // e.g., camera is centered left-to-right

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initVision();

        // Speed up tag detection by reducing decimation if tags are close
        aprilTag.setDecimation(2);

        telemetry.addData("Status", "Initialized. Manual Exposure Set.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            boolean targetFound = false;

            for (AprilTagDetection detection : currentDetections) {
                // Focus on Goal Tags: 20 (Blue) and 24 (Red)
                if (detection.metadata != null && (detection.id == 20 || detection.id == 24)) {

                    RobotPose pose = calculateRobotPose(detection);

                    if (pose != null) {
                        targetFound = true;
                        telemetry.addLine("\n--- Robot Field Position ---");
                        telemetry.addData("Visible Tag", "ID %d (%s)", detection.id, detection.metadata.name);
                        telemetry.addData("X (Inches)", "%.2f", pose.x);
                        telemetry.addData("Y (Inches)", "%.2f", pose.y);
                        telemetry.addData("Heading", "%.2f°", pose.heading);
                    }
                }
            }

            if (!targetFound) telemetry.addLine("No Goal Tags in sight...");
            telemetry.update();
            sleep(20);
        }
        visionPortal.close();
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .build();

        // Optional: Set manual exposure to reduce motion blur
        setManualExposure(6, 250);
    }

    public RobotPose calculateRobotPose(AprilTagDetection detection) {
        if (detection.ftcPose == null) return null;

        double tagX, tagY, tagBearing;

        // Tag Field Positions (Example values - verify with official field map)
        if (detection.id == 24) { // Red Goal
            tagX = 72.0; tagY = 72.0; tagBearing = 225.0;
        } else if (detection.id == 20) { // Blue Goal
            tagX = -72.0; tagY = 72.0; tagBearing = 315.0;
        } else {
            return null;
        }

        // 1. Calculate Robot Heading
        // Heading = Tag's Field Orientation - 180 (to look back at robot) - Detection Yaw
        double robotHeading = AngleUnit.normalizeDegrees(tagBearing - 180 - detection.ftcPose.yaw);

        // 2. Adjust for Camera Offset relative to Robot Center
        // We subtract the camera offset because the FTC pose is "Camera to Tag"
        double camRelX = detection.ftcPose.x - CAMERA_LEFT_OFFSET;
        double camRelY = detection.ftcPose.y + CAMERA_FORWARD_OFFSET;

        // 3. Transform Relative Camera Coordinates to Global Field Coordinates
        // The angle alpha represents the rotation needed to align camera space with field space
        double alpha = Math.toRadians(robotHeading);

        // Rotation Matrix: Rotate the "Camera to Tag" vector by the robot's heading
        double fieldRelativeX = camRelX * Math.cos(alpha) - camRelY * Math.sin(alpha);
        double fieldRelativeY = camRelX * Math.sin(alpha) + camRelY * Math.cos(alpha);

        // 4. Final Robot Position = Tag Position - Field-Relative Vector
        double robotX = tagX - fieldRelativeX;
        double robotY = tagY - fieldRelativeY;

        return new RobotPose(robotX, robotY, robotHeading);
    }

    // Boilerplate for consistent camera performance
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    public static class RobotPose {
        public double x, y, heading;
        public RobotPose(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }
    }
}
