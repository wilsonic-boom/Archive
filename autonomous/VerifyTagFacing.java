package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@Autonomous(name = "VerifyTagFacing")
public class VerifyTagFacing extends LinearOpMode {

    // Drivetrain motors
    DcMotor lf, rf, lb, rb;

    // Vision objects
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;

    // Alignment parameters
    double YAW_TOLERANCE = 2.0;
    double TURN_POWER = 0.2;

    @Override
    public void runOpMode() {

        // Map motors
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        // Reverse right motors
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        // Create AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // Start vision system
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Ready to align");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            alignToTag();
            telemetry.update();
        }
    }

    /**
     * Detects the first visible AprilTag and turns the robot
     * until the yaw error is within YAW_TOLERANCE degrees.
     * Stops motors if no tag is detected or already aligned.
     */
    private void alignToTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        double turnPowerLeft = 0;
        double turnPowerRight = 0;

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            double yawError = tag.ftcPose.yaw;

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Yaw Error", yawError);

            if (yawError > YAW_TOLERANCE) {
                // Tag is to the right — turn left
                turnPowerLeft = TURN_POWER;
                turnPowerRight = -TURN_POWER;
            } else if (yawError < -YAW_TOLERANCE) {
                // Tag is to the left — turn right
                turnPowerLeft = -TURN_POWER;
                turnPowerRight = TURN_POWER;
            }
            // else: within tolerance, powers stay 0 (aligned)

        } else {
            telemetry.addLine("No Tag Detected – idle");
        }

        // Apply motor powers
        lf.setPower(turnPowerLeft);
        lb.setPower(turnPowerLeft);
        rf.setPower(turnPowerRight);
        rb.setPower(turnPowerRight);
    }
}
