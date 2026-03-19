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

    // --- CONFIGURATION ---
    // Ensure "Webcam 1" matches your Robot Configuration on the Driver Station
    private static final String WEBCAM_NAME = "Webcam 1";

    // Physical calibration: distance from robot center to camera lens (inches)
    // Measure these from the robot's center of rotation to the camera lens
    private static final double CAMERA_FORWARD_OFFSET = 0.0; // positive = camera is in front of center
    private static final double CAMERA_LEFT_OFFSET    = 0.0; // positive = camera is left of center

    // Camera intrinsics for your specific webcam — get these from the FTC Camera Calibration tool
    // or your camera's calibration file. Leaving at 0 disables this correction.
    // Example values for a Logitech C920: fx=622.001, fy=622.001, cx=319.803, cy=241.251
    private static final double LENS_FX = 0;
    private static final double LENS_FY = 0;
    private static final double LENS_CX = 0;
    private static final double LENS_CY = 0;

    // --- VISION OBJECTS ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        // 1. Initialize the AprilTag Processor
        // Forces units to Inches/Degrees so the math equations work correctly
        AprilTagProcessor.Builder aprilTagBuilder = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);

        // Only apply lens intrinsics if values have been provided
        if (LENS_FX != 0 && LENS_FY != 0 && LENS_CX != 0 && LENS_CY != 0) {
            aprilTagBuilder.setLensIntrinsics(LENS_FX, LENS_FY, LENS_CX, LENS_CY);
        }

        aprilTag = aprilTagBuilder.build();

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
     * Inner class to hold the calculated robot location on the field.
     * X and Y are in inches on a 144x144 grid with (0,0) at the field center.
     * Heading is the robot's orientation in degrees (field frame).
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
     * CORE CALCULATION: Transforms a tag detection into global X, Y coordinates.
     *
     * Key fix: ftcPose describes where the TAG is relative to the ROBOT (camera).
     * To find where the ROBOT is relative to the TAG, we negate the vector before
     * rotating it into the field coordinate frame.
     */
    public RobotPose calculateRobotPose(AprilTagDetection detection) {
        if (detection.ftcPose == null) return null;

        double tagX, tagY, tagAngle;

        // Field coordinates for a 144x144 grid with (0,0) at the center.
        // Verify these against the Decode season game manual — they can change yearly.
        // Red Goal (24) is Top-Right corner, facing South-West (225°).
        // Blue Goal (20) is Top-Left corner, facing South-East (315°).
        if (detection.id == 24) {
            tagX = 72.0; tagY = 72.0; tagAngle = 225.0;
        } else if (detection.id == 20) {
            tagX = -72.0; tagY = 72.0; tagAngle = 315.0;
        } else {
            return null;
        }

        // Convert the tag's facing direction to radians for the rotation matrix
        double alpha = Math.toRadians(tagAngle);

        // Compensate for camera mount position relative to robot center
        double adjY = detection.ftcPose.y + CAMERA_FORWARD_OFFSET;
        double adjX = detection.ftcPose.x + CAMERA_LEFT_OFFSET;

        // Negate the vector: ftcPose gives (camera → tag), but we need (tag → robot)
        double vecX = -adjX;
        double vecY = -adjY;

        // Standard 2D rotation matrix: rotate the tag→robot vector into the field frame
        double rotatedX = (vecX * Math.cos(alpha)) - (vecY * Math.sin(alpha));
        double rotatedY = (vecX * Math.sin(alpha)) + (vecY * Math.cos(alpha));

        // Translation: robot's field position = tag position + rotated relative vector
        double robotX = tagX + rotatedX;
        double robotY = tagY + rotatedY;

        // Heading: derive robot orientation from the tag's known facing angle and camera yaw.
        // If reported heading is 180° off from reality, flip the sign on ftcPose.yaw.
        double robotHeading = AngleUnit.normalizeDegrees(tagAngle - detection.ftcPose.yaw - 180);

        return new RobotPose(robotX, robotY, robotHeading);
    }
}
