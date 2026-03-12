package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="AprilTag Linear Test")
public class AprilTagLinear extends LinearOpMode {

    // Tag IDs for each alliance goal
    private static final int TAG_BLUE_GOAL = 21;
    private static final int TAG_RED_GOAL  = 22;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    // Stores the robot's position and heading on the field
    public static class FieldPose {
        public double x;
        public double y;
        public double heading;

        public FieldPose(double x, double y, double heading) {
            this.x       = x;
            this.y       = y;
            this.heading = heading;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initAprilTagPipeline(hardwareMap, telemetry);

        telemetry.addLine("AprilTag pipeline initialized.");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        // Main loop: detect tags and compute robot field pose each cycle
        while (opModeIsActive()) {

            updateDetections();

            FieldPose robotPose = null;
            AprilTagDetection usedTag = null;

            for (int id : new int[]{TAG_BLUE_GOAL, TAG_RED_GOAL}) {
                AprilTagDetection tag = getTagById(id);
                if (tag != null && tag.ftcPose != null) {
                    robotPose = getRobotFieldPose(tag);
                    if (robotPose != null) {
                        usedTag = tag;
                        break;
                    }
                }
            }

            displayDetectionTelemetry(usedTag, robotPose);
            telemetry.update();
        }

        stopAprilTag();
    }

    // Initialises the AprilTag processor and vision portal
    private void initAprilTagPipeline(HardwareMap hwmap, Telemetry telemetry) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwmap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    // Pulls the latest detections from the processor
    private void updateDetections() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    // Returns the detection matching the given tag ID, or null if not found
    private AprilTagDetection getTagById(int id) {
        for (AprilTagDetection tag : detectedTags) {
            if (tag.id == id) return tag;
        }
        return null;
    }

    // Outputs tag pose and computed robot field position to telemetry
    private void displayDetectionTelemetry(AprilTagDetection tag, FieldPose robotPose) {

        if (tag == null) {
            telemetry.addLine("No goal tag detected.");
            return;
        }

        String goalName = (tag.id == TAG_BLUE_GOAL) ? "BLUE GOAL" : "RED GOAL";

        if (tag.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s  [%s]",
                    tag.id, tag.metadata.name, goalName));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown  [%s]",
                    tag.id, goalName));
        }

        telemetry.addLine(String.format("XYZ: %.1f, %.1f, %.1f (cm)",
                tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
        telemetry.addLine(String.format("PRY: %.1f, %.1f, %.1f (deg)",
                tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
        telemetry.addLine(String.format("RBE: %.1f (cm), %.1f (deg), %.1f (deg)",
                tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));

        if (robotPose != null) {
            telemetry.addLine("---- ROBOT FIELD POSE ----");
            telemetry.addLine(String.format("Field X: %.1f cm  (%.1f in)", robotPose.x, robotPose.x / 2.54));
            telemetry.addLine(String.format("Field Y: %.1f cm  (%.1f in)", robotPose.y, robotPose.y / 2.54));
            telemetry.addLine(String.format("Heading: %.1f deg", robotPose.heading));
        }
    }

    // Returns the known field position and heading of each goal tag in cm
    private FieldPose getAprilTagFieldPose(int tagId) {
        final double IN_TO_CM = 2.54;
        switch (tagId) {
            case TAG_BLUE_GOAL:
                return new FieldPose(-72 * IN_TO_CM,  72 * IN_TO_CM, -90.0);
            case TAG_RED_GOAL:
                return new FieldPose( 72 * IN_TO_CM,  72 * IN_TO_CM,  90.0);
            default:
                return null;
        }
    }

    // Converts the camera-relative tag pose into the robot's position on the field
    private FieldPose getRobotFieldPose(AprilTagDetection tag) {

        FieldPose tagPose = getAprilTagFieldPose(tag.id);
        if (tagPose == null || tag.ftcPose == null) return null;

        double robotXRel = tag.ftcPose.x;
        double robotYRel = tag.ftcPose.y;

        double headingRad = Math.toRadians(tagPose.heading);

        double rotX = robotXRel * Math.cos(headingRad) - robotYRel * Math.sin(headingRad);
        double rotY = robotXRel * Math.sin(headingRad) + robotYRel * Math.cos(headingRad);

        double fieldX = tagPose.x - rotX;
        double fieldY = tagPose.y - rotY;

        double fieldHeading = tagPose.heading - tag.ftcPose.yaw;

        return new FieldPose(fieldX, fieldY, fieldHeading);
    }

    // Closes the vision portal when the op mode ends
    private void stopAprilTag() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
