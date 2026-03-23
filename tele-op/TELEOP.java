package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class Teleop extends LinearOpMode {

    boolean Slow = true;

    boolean reverse = false;

    private DcMotorEx Shooter = null;


    boolean intake = false;

    private AprilTagProcessor aprilTag;
    private VisionPortal       visionPortal;

    // ── Camera resolution — match your webcam's supported resolution ──────────
    private static final int CAM_WIDTH  = 640;
    private static final int CAM_HEIGHT = 480;


    public static double velocity(double x, double y) {
        return (44.297 * Math.pow(Math.abs(y), 1.0 / 3.0)
                - 16.893 * Math.abs(x)
                + 0.0018285 * Math.pow(Math.abs(y), 2)
                + 5.0046e-05 * Math.pow(Math.abs(x), 2) * Math.pow(y, 2)
                + 1460); // * (TICKS_PER_REV / 60.0); // maybe the  * (TICKS_PER_REV / 60.0) not needed?
    }

    public static double hoodServoCalc(double x, double y) {
        return 0.037524 * Math.pow(Math.abs(y), 0.5)
                - 0.027099 * Math.pow(Math.abs(x), 1.0 / 3.0)
                - 0.0051773 * Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))
                + 0.18998;
    }

    public static double calculateVelocity(double range) {
        return 8.8912e8 * Math.pow(range, -4)
                - 1.7069e6 * Math.pow(range, -2)
                + 0.0012772 * Math.pow(range, 2) * Math.log(range)
                + 1594.1;
    }

    public static double calculateHoodServo(double range) {
        return 1.7043e5 * Math.pow(range, -4)
                - 26.383 * Math.pow(range, -1)
                - 0.0023591 * range
                + 0.69062;
    }


    public static double distance_formula(double x1, double y1, double x2, double y2) {
        return Math.sqrt(
                Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2)
        );
    }

    @Override
    public void runOpMode() {

        // Make sure your ID's match your configuration
        DcMotor FLmotor = hardwareMap.dcMotor.get("FLmotor");
        DcMotor BLmotor = hardwareMap.dcMotor.get("BLmotor");
        DcMotor FRmotor = hardwareMap.dcMotor.get("FRmotor");
        DcMotor BRmotor = hardwareMap.dcMotor.get("BRmotor");

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        DcMotor IntakeMain = hardwareMap.dcMotor.get("IntakeMain");
        DcMotor IntakeControl = hardwareMap.dcMotor.get("IntakeControl");

        Servo hood = hardwareMap.get(Servo.class, "Hood");
        Servo gate = hardwareMap.get(Servo.class, "Gate");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMain.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

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


        waitForStart();


        IntakeControl.setPower(0);
        IntakeMain.setPower(0);

        IntakeControl.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("ControlIntake", "forward");

        hood.setPosition(0);
        gate.setPosition(0.8);

        imu.resetYaw();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            if (Math.abs(rx) < 0.1) {
                rx = 0;
            }

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double FLpower = (rotY + rotX + rx) / denominator;
            double BLpower = (rotY - rotX + rx) / denominator;
            double FRpower = (rotY - rotX - rx) / denominator;
            double BRpower = (rotY + rotX - rx) / denominator;

            if (gamepad1.rightStickButtonWasPressed()) {
                if (Slow) {
                    Slow = false;
                } else {
                    Slow = true;
                }
            }

            if (Slow) {
                FLmotor.setPower(FLpower);
                BLmotor.setPower(BLpower);
                FRmotor.setPower(FRpower);
                BRmotor.setPower(BRpower);
                telemetry.addData("Speed", "Normal");
            } else {
                FLmotor.setPower(FLpower / 2);
                BLmotor.setPower(BLpower / 2);
                FRmotor.setPower(FRpower / 2);
                BRmotor.setPower(BRpower / 2);
                telemetry.addData("Speed", "Slow");
            }


            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {
                for (AprilTagDetection tag : detections) {
                    telemetry.addLine("  ── Metric equivalents ──");
                    telemetry.addData("  Range",    "%.3f", tag.ftcPose.range);
                    telemetry.addData("  X",        "%.3f", tag.ftcPose.x);
                    telemetry.addData("  Y",        "%.3f", tag.ftcPose.y);
                    telemetry.addData("  Z",        "%.3f", tag.ftcPose.z);
                    telemetry.addData("  2D Range",        "%.3f", Math.sqrt(Math.pow(tag.ftcPose.range, 2) + Math.pow(tag.ftcPose.z, 2)));
                }
            }



            // SHOOTER
            if (gamepad1.right_trigger == 0) {
                Shooter.setPower(-gamepad1.left_trigger);
            } else {
                Shooter.setPower(gamepad1.right_trigger);
            }

            telemetry.addData("SHOOTER", Shooter.getVelocity());


            //AUTO SHOOT
            if (gamepad1.right_bumper) {
                double targetTicksPerSec = velocity(FieldX, FieldY);
                double targetServo = hoodServoCalc(FieldX, FieldY);
                targetServo = Math.max(0.0, Math.min(1.0, targetServo));
                Shooter.setVelocity(targetTicksPerSec);
                hood.setPosition(targetServo);
                sleep(750);
                gate.setPosition(0.8);
                IntakeControl.setPower(-1);
                sleep(500);
                IntakeControl.setPower(0);
                gate.setPosition(1);
                sleep(100);
            }

            if (gamepad1.dpad_up) {
                double change = hood.getPosition() + 0.01;
                if (change > 0.20) {
                    change = 0.20;
                }
                hood.setPosition(change);
            } else if (gamepad1.dpad_down) {
                hood.setPosition(hood.getPosition() - 0.01);
            }

            // INTAKE

            if (gamepad1.crossWasPressed()) {
                if (reverse == true) {
                    reverse = false;
                } else {
                    reverse = true;
                }
            }

            if (reverse) {
                IntakeControl.setDirection(DcMotorSimple.Direction.FORWARD);
                telemetry.addData("IntakeControl", "forward");
            } else {
                IntakeControl.setDirection(DcMotorSimple.Direction.REVERSE);
                telemetry.addData("IntakeControl", "backward");
            }


            if (gamepad1.squareWasPressed()) {
                if (intake == true) {
                    intake = false;
                } else {
                    intake = true;
                }
            }

            if (intake) {
                IntakeControl.setPower(1);
                IntakeMain.setPower(1);
            } else {
                IntakeControl.setPower(0);
                IntakeMain.setPower(0);
            }


            if (gamepad1.circleWasPressed()) {
                if (gate.getPosition() == 0.8) {
                    gate.setPosition(1);
                } else {
                    gate.setPosition(0.8);
                }
            }


            // EMERGENCY STOP

            if (gamepad1.share) {
                requestOpModeStop();
            }

            if (gamepad1.leftBumperWasPressed()) {
                if (!detections.isEmpty()) {
                    double rangeSeen = -1;
                    for (AprilTagDetection tag : detections) {
                        // Only process tags with ID 24
                        if (tag.id == 24 || tag.id == 20) {
                            rangeSeen = Math.sqrt(Math.pow(tag.ftcPose.range, 2) + Math.pow(tag.ftcPose.z, 2));
                        }
                    }

                    if (!(rangeSeen == -1)) {
                        double targetTicksPerSec = calculateVelocity(rangeSeen);
                        double targetServo = calculateHoodServo(rangeSeen);
                        targetServo = Math.max(0.0, Math.min(1.0, targetServo));
                        Shooter.setVelocity(targetTicksPerSec);
                        hood.setPosition(targetServo);
                        sleep(750);
                        gate.setPosition(0.8);
                        IntakeControl.setPower(-1);
                        sleep(500);
                        IntakeControl.setPower(0);
                        gate.setPosition(1);
                        sleep(100);
                    }
                }
            }

            if (gamepad1.rightBumperWasPressed()) {

                Shooter.setVelocity(Shooter.getVelocity());

                gate.setPosition(0.8);
                sleep(500); // change
                gate.setPosition(1);

                IntakeControl.setDirection(DcMotorSimple.Direction.REVERSE);
                sleep(500);
                gate.setPosition(0.8);
                sleep(500);
                gate.setPosition(1);
                sleep(500);
                gate.setPosition(0.8);
                sleep(500);
            }

            telemetry.addData("GATE", gate.getPosition());
            telemetry.update();
        }
    }
}


