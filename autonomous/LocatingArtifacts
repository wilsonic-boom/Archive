package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;

import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Decode_Master_Locator", group = "Autonomous")
public class LocatingArtifacts extends LinearOpMode {
    // initialising hardware
    DcMotor leftDrive;
    DcMotor rightDrive;

    VisionPortal visionPortal;
    ColorBlobLocatorProcessor greenLocator;
    ColorBlobLocatorProcessor purpleLocator;
    // constants needed for alignment and slow down when approaching an artifact
    double SCREEN_CENTER_X = 320.0;
    double TURN_P = 0.004;

    double SLOW_AREA = 4000;
    double STOP_AREA = 9000;

    double MAX_SPEED = 0.30;
    double MIN_SPEED = 0.10;

    @Override
    public void runOpMode() {

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        // locating green artifacts
        greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setBlurSize(5)
                .setDrawContours(true)
                .build();

        ColorRange purpleHSV = new ColorRange(
                ColorSpace.HSV,
                new Scalar(125, 70, 50),
                new Scalar(165, 255, 255)
        );
        // locating purple artifacts
        purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(purpleHSV)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setBlurSize(5)
                .setDrawContours(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(greenLocator)
                .addProcessor(purpleLocator)
                .build();

        telemetry.addLine("Vision ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();
            blobs.addAll(greenLocator.getBlobs());
            blobs.addAll(purpleLocator.getBlobs());

            ColorBlobLocatorProcessor.Blob target = findLargestBlob(blobs);

            if (target != null) {

                double x = target.getBoxFit().center.x;
                double error = x - SCREEN_CENTER_X;
                double area = target.getContourArea();

                telemetry.addData("Blob area", area);

                if (area >= STOP_AREA) {
                    stopRobot();
                    telemetry.addLine("Target reached");
                }
                else if (Math.abs(error) > 15) {
                    double steer = Range.clip(error * TURN_P, -0.3, 0.3);
                    moveRobot(steer, -steer);
                    telemetry.addLine("Aligning to target"); // logic for alignment
                }
                else {
                    double speed = calculateApproachSpeed(area);
                    moveRobot(speed, speed);
                    telemetry.addData("Drive speed", speed);
                    telemetry.addLine("Approaching target");
                }
            }
            else {
                moveRobot(0.15, -0.15);
                telemetry.addLine("Searching for target");
            }

            telemetry.update();
        }

        visionPortal.close();
    }

    double calculateApproachSpeed(double area) {

        if (area <= SLOW_AREA) {
            return MAX_SPEED;
        }

        if (area >= STOP_AREA) {
            return MIN_SPEED;
        }

        double ratio = (STOP_AREA - area) / (STOP_AREA - SLOW_AREA);
        return Range.clip(
                MIN_SPEED + ratio * (MAX_SPEED - MIN_SPEED),
                MIN_SPEED,
                MAX_SPEED
        );
    }

    ColorBlobLocatorProcessor.Blob findLargestBlob(List<ColorBlobLocatorProcessor.Blob> blobs) {

        ColorBlobLocatorProcessor.Blob bestBlob = null;
        double largestArea = 0;

        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            double area = blob.getContourArea();
            if (area > 400 && area > largestArea) {
                largestArea = area;
                bestBlob = blob;
            }
        }
        return bestBlob;
    }

    void moveRobot(double leftPower, double rightPower) {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    void stopRobot() {
        moveRobot(0, 0);
    }
}
