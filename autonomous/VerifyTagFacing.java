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
    double YAW_TOLERANCE = 2.0;   // This is essentially the margin of error!
    double TURN_POWER = 0.2;      // fixed turn speed

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

            // Get detected tags
            List<AprilTagDetection> detections = aprilTag.getDetections();

            double turnPowerLeft = 0;
            double turnPowerRight = 0;

            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);
                double yawError = tag.ftcPose.yaw;

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Yaw Error", yawError);

                // Turn left or right at fixed speed until aligned
                if (yawError > YAW_TOLERANCE) {
                    turnPowerLeft = TURN_POWER;
                    turnPowerRight = -TURN_POWER;
                } else if (yawError < -YAW_TOLERANCE) {
                    turnPowerLeft = -TURN_POWER;
                    turnPowerRight = TURN_POWER;
                } else {
                    // Aligned so stop turning
                    turnPowerLeft = 0;
                    turnPowerRight = 0;
                }

            } else {
                telemetry.addLine("No Tag Detected – idle");
                turnPowerLeft = 0;
                turnPowerRight = 0;
            }

            // Apply motor powers for turning
            lf.setPower(turnPowerLeft);
            lb.setPower(turnPowerLeft);
            rf.setPower(turnPowerRight);
            rb.setPower(turnPowerRight);

            telemetry.update();
        }
    }
}

/* What this code does:
    Gets list of detected tags in the list if length of list > 1
    Retrieves the first element
    Obtains metrics from the april tag and turns in the right direction until facing it
    If it is aligned make the robot still
    Set power to all the motors for alignment to happen
 */