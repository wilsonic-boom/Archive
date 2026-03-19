package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "april_tag_test", group = "Linear OpMode")
public class april_tag_test extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal       visionPortal;

    // ── Camera resolution — match your webcam's supported resolution ──────────
    private static final int CAM_WIDTH  = 640;
    private static final int CAM_HEIGHT = 480;

    @Override
    public void runOpMode() {

        // ── Build the AprilTag processor ──────────────────────────────────────
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)             // draws XYZ axes on tag in camera stream
                .setDrawCubeProjection(true)   // draws a cube on the tag
                .setDrawTagOutline(true)        // outlines the detected tag
                .build();

        // ── Build the Vision Portal (attaches processor to the webcam) ────────
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(CAM_WIDTH, CAM_HEIGHT))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized — waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections.isEmpty()) {
                telemetry.addData("AprilTags", "None detected");

            } else {
                telemetry.addData("Tags found", detections.size());

                for (AprilTagDetection tag : detections) {
                    telemetry.addLine();

                    // ── Tag identity ──────────────────────────────────────────
                    if (tag.metadata != null) {
                        // Known tag — has metadata from the field layout file
                        telemetry.addData("  Tag ID",    "%d  (%s)", tag.id, tag.metadata.name);
                    } else {
                        // Unknown tag — still gives raw pose data
                        telemetry.addData("  Tag ID",    "%d  (unknown — not in tag library)", tag.id);
                    }

                    // ── Pose data (ftcPose is relative to the CAMERA) ─────────
                    // All distances in INCHES by default (FTC SDK convention).
                    // Divide by 39.3701 to convert to metres if preferred.
                    if (tag.ftcPose != null) {
                        telemetry.addData("  Range",    "%.2f in  (straight-line dist to tag)", tag.ftcPose.range);
                        telemetry.addData("  Bearing",  "%.2f °   (left/right angle to tag)",   tag.ftcPose.bearing);
                        telemetry.addData("  Elevation","%.2f °   (up/down angle to tag)",       tag.ftcPose.elevation);

                        telemetry.addLine("  ── Cartesian (camera frame) ──");
                        telemetry.addData("  X (strafe)", "%.2f in  (+= tag is RIGHT of camera)", tag.ftcPose.x);
                        telemetry.addData("  Y (forward)","%.2f in  (+= tag is IN FRONT of camera)", tag.ftcPose.y);
                        telemetry.addData("  Z (height)", "%.2f in  (+= tag is ABOVE camera)",    tag.ftcPose.z);
//                        Math.pow(Math.pow(tag.ftcPose.range, 2) - Math.pow(tag.ftcPose.z, 2), 0.5)
//                        telemetry.addLine("  ── thajan guess");
//                        telemetry.addData("  X (strafe)", "%.2f in  (+= tag is RIGHT of camera)", 60.0-tag.ftcPose.x);
//                        telemetry.addData("  Y (forward)","%.2f in  (+= tag is IN FRONT of camera)", 60.0-tag.ftcPose.y);

                        telemetry.addLine("  ── Rotation (camera frame) ──");
                        telemetry.addData("  Yaw",   "%.2f °  (tag rotation left/right)", tag.ftcPose.yaw);
                        telemetry.addData("  Pitch", "%.2f °  (tag rotation up/down)",    tag.ftcPose.pitch);
                        telemetry.addData("  Roll",  "%.2f °  (tag rotation clockwise)",  tag.ftcPose.roll);

                        // ── Convenience: metric equivalents ──────────────────
                        telemetry.addLine("  ── Metric equivalents ──");
                        telemetry.addData("  Range (m)",    "%.3f", tag.ftcPose.range   / 39.3701);
                        telemetry.addData("  X (m)",        "%.3f", tag.ftcPose.x       / 39.3701);
                        telemetry.addData("  Y (m)",        "%.3f", tag.ftcPose.y       / 39.3701);
                        telemetry.addData("  Z (m)",        "%.3f", tag.ftcPose.z       / 39.3701);

                    } else {
                        // ftcPose is null when the tag was detected but pose
                        // could not be solved (e.g. tag partially off-screen)
                        telemetry.addData("  Pose", "Not available (tag partially visible?)");
                    }

                    // ── Raw corner pixel positions (useful for calibration) ───
                    telemetry.addLine("  ── Raw pixel corners ──");
                    telemetry.addData("  Center px", "(%.0f,  %.0f)", tag.center.x, tag.center.y);
                    telemetry.addData("  TL corner", "(%.0f,  %.0f)", tag.corners[0].x, tag.corners[0].y);
                    telemetry.addData("  TR corner", "(%.0f,  %.0f)", tag.corners[1].x, tag.corners[1].y);
                    telemetry.addData("  BR corner", "(%.0f,  %.0f)", tag.corners[2].x, tag.corners[2].y);
                    telemetry.addData("  BL corner", "(%.0f,  %.0f)", tag.corners[3].x, tag.corners[3].y);

                    // ── Decision distance stamp for logging ───────────────────
                    telemetry.addData("  Hamming err", tag.hamming); // 0 = perfect decode
                    telemetry.addData("  Decision margin", "%.2f  (higher = more confident)", tag.decisionMargin);
                }
            }

            telemetry.update();
        }

        // Release camera on stop
        visionPortal.close();
    }
}