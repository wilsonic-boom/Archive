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

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    // getting field position using coords from april tag
    public static class FieldPose {
        public double x;
        public double y;
        public double heading;

        public FieldPose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
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

        while (opModeIsActive()) {

            updateDetections();

            AprilTagDetection tag20 = getTagById(20);

            displayDetectionTelemetry(tag20);

            telemetry.update();
        }

        stopAprilTag();
    }

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

    private void updateDetections() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    private AprilTagDetection getTagById(int id) {
        for (AprilTagDetection tag : detectedTags) {
            if (tag.id == id) return tag;
        }
        return null;
    }

    private void displayDetectionTelemetry(AprilTagDetection tag) {

        if (tag == null) {
            telemetry.addLine("Tag not found.");
            return;
        }

        if (tag.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", tag.id, tag.metadata.name));
            telemetry.addLine(String.format("XYZ: %.1f, %.1f, %.1f (cm)",
                    tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
            telemetry.addLine(String.format("PRY: %.1f, %.1f, %.1f (deg)",
                    tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
            telemetry.addLine(String.format("RBE: %.1f (cm), %.1f (deg), %.1f (deg)",
                    tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));

            // ================== ADDITION ==================
            FieldPose robotPose = getRobotFieldPose(tag);
            if (robotPose != null) {
                telemetry.addLine("---- ROBOT FIELD POSE ----");
                telemetry.addLine(String.format("Field X: %.1f cm", robotPose.x));
                telemetry.addLine(String.format("Field Y: %.1f cm", robotPose.y));
                telemetry.addLine(String.format("Heading: %.1f deg", robotPose.heading));
            }
            //

        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", tag.id));
            telemetry.addLine(String.format("Center: %.0f, %.0f (px)", tag.center.x, tag.center.y));
        }
    }

    // method to find position
    private FieldPose getAprilTagFieldPose(int tagId) {
        // REPLACE with official FTC Decode field coordinates
        switch (tagId) {
            case 20:
                return new FieldPose(0, 144, 180);
            case 21:
                return new FieldPose(-72, 144, 180);
            case 22:
                return new FieldPose(72, 144, 180);
            default:
                return null;
        }
    }

    private FieldPose getRobotFieldPose(AprilTagDetection tag) {

        FieldPose tagPose = getAprilTagFieldPose(tag.id);
        if (tagPose == null || tag.ftcPose == null) return null;

        double robotXRel = tag.ftcPose.x;
        double robotYRel = tag.ftcPose.y;

        double headingRad = Math.toRadians(tagPose.heading);

        double fieldX = tagPose.x
                - (robotXRel * Math.cos(headingRad)
                - robotYRel * Math.sin(headingRad));

        double fieldY = tagPose.y
                - (robotXRel * Math.sin(headingRad)
                + robotYRel * Math.cos(headingRad));

        double fieldHeading = tagPose.heading - tag.ftcPose.yaw;

        return new FieldPose(fieldX, fieldY, fieldHeading);
    }


    private void stopAprilTag() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
