package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "AprilTag Linear Localization")
public class AprilTagLinear extends LinearOpMode {

    private static final String WEBCAM_NAME = "Webcam 1";

    // Physical calibration: distance from robot center to camera lens (inches)
    private static final double CAMERA_FORWARD_OFFSET = 0.0;
    private static final double CAMERA_LEFT_OFFSET = 0.0;

    // objects for vision
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        // 1. Initialize the AprilTag Processor
        // Forces units to Inches/Degrees so the math equations work correctly
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // 2. Initialize the Vision Portal (Camera)
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized. Using Tags 20 (Blue) and 24 (Red).");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            boolean targetFound = false;

            for (AprilTagDetection detection : currentDetections) {
                // Focus strictly on the Goal tags as specified
                if (detection.id == 20 || detection.id == 24) {

                    RobotPose pose = calculateRobotPose(detection);

                    if (pose != null) {
                        targetFound = true;
                        telemetry.addLine("\n--- Robot Field Position ---");
                        telemetry.addData("Target Tag", "ID %d", detection.id);
                        telemetry.addData("X (Inches)", "%.2f", pose.x);
                        telemetry.addData("Y (Inches)", "%.2f", pose.y);
                        telemetry.addData("Heading", "%.2f°", pose.heading);
                    }
                }
            }

            if (!targetFound) {
                telemetry.addLine("No Goal Tags in sight...");
            }

            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }

    /**
     * Inner class to hold the calculated robot location
     */
    public static class RobotPose {
        public double x, y, heading;
        public RobotPose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    /**
     * CORE CALCULATION: Transforms a slanted tag detection into global X, Y coordinates.
     */
    public RobotPose calculateRobotPose(AprilTagDetection detection) {
        if (detection.ftcPose == null) return null;

        double tagX, tagY, tagAngle;

        // Red Goal (24) is Top-Right facing South-West (225°).
        // Blue Goal (20) is Top-Left facing South-East (315°).
        if (detection.id == 24) {
            tagX = 72.0; tagY = 72.0; tagAngle = 225.0;
        } else if (detection.id == 20) {
            tagX = -72.0; tagY = 72.0; tagAngle = 315.0;
        } else {
            return null;
        }

        // Convert the "facing direction" of the tag to radians for trig
        double alpha = Math.toRadians(tagAngle);

        // Compensate for camera mount position relative to robot center
        double adjY = detection.ftcPose.y + CAMERA_FORWARD_OFFSET;
        double adjX = detection.ftcPose.x + CAMERA_LEFT_OFFSET;

        // Rotation Matrix: Projects the camera's relative vector onto the grid axes.
        // Corrects for the 45-degree angle of the goal corners.
        double rotatedX = (adjY * Math.cos(alpha)) - (adjX * Math.sin(alpha));
        double rotatedY = (adjY * Math.sin(alpha)) + (adjX * Math.cos(alpha));

        // Translation: Find robot position by adding the rotated relative vector to the Tag position.
        double robotX = tagX + rotatedX;
        double robotY = tagY + rotatedY;

        // Heading: Compare Tag direction to Camera Yaw to find robot field orientation.
        double robotHeading = AngleUnit.normalizeDegrees(tagAngle - detection.ftcPose.yaw - 180);

        return new RobotPose(robotX, robotY, robotHeading);
    }
}
