package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "shooter_test", group = "Linear OpMode")
public class shooter_test extends LinearOpMode {

    private DcMotorEx shooterMotor;
    private Servo     hoodServo;

    // ── Tunable constants ─────────────────────────────────────────────────────
    private static final double TICKS_PER_REV   = 50;
    private static final double SERVO_MIN        = 0;
    private static final double SERVO_MAX        = 0.2;
    private static final double SERVO_STEP       = 0.01;
    private static final double RPM_STEP         = 100.0;
    private static final double RPM_MIN          = 0.0;
    private static final double RPM_MAX          = 2000.0;

    // ── State ─────────────────────────────────────────────────────────────────
    private double  servoPos      = 0.0;   // start at minimum
    private double  targetRPM     = 0.0;
    private boolean motorRunning  = false;

    // Button edge-detection flags (true = was pressed last loop)
    private boolean lastDpadUp    = false;
    private boolean lastDpadDown  = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft  = false;
    private boolean lastRB        = false;

    @Override
    public void runOpMode() {

        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
//        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
//        shooterMotor.setVelocity(0);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setPosition(servoPos);

        telemetry.addData("Status", "Initialized — waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ── Edge detection helpers ────────────────────────────────────────
            boolean dpadUpNow    = gamepad1.dpad_up;
            boolean dpadDownNow  = gamepad1.dpad_down;
            boolean dpadRightNow = gamepad1.dpad_right;
            boolean dpadLeftNow  = gamepad1.dpad_left;
            boolean rbNow        = gamepad1.right_bumper;

            boolean pressedUp    = dpadUpNow    && !lastDpadUp;
            boolean pressedDown  = dpadDownNow  && !lastDpadDown;
            boolean pressedRight = dpadRightNow && !lastDpadRight;
            boolean pressedLeft  = dpadLeftNow  && !lastDpadLeft;
            boolean pressedRB    = rbNow        && !lastRB;

            // ── dpad up/down → servo position ─────────────────────────────────
            if (pressedUp) {
                servoPos = Math.min(servoPos + SERVO_STEP, SERVO_MAX);
                hoodServo.setPosition(servoPos);
            }
            if (pressedDown) {
                servoPos = Math.max(servoPos - SERVO_STEP, SERVO_MIN);
                hoodServo.setPosition(servoPos);
            }

            // ── dpad right/left → target RPM ──────────────────────────────────
            if (pressedRight) {
                targetRPM = Math.min(targetRPM + RPM_STEP, RPM_MAX);
            }
            if (pressedLeft) {
                targetRPM = Math.max(targetRPM - RPM_STEP, RPM_MIN);
            }

            // ── right bumper → toggle motor ───────────────────────────────────
            if (pressedRB) {
                motorRunning = !motorRunning;
                if (!motorRunning) {
                    shooterMotor.setVelocity(0);
                }
            }

            if (motorRunning) {
                double ticksPerSec = (targetRPM * TICKS_PER_REV) / 60.0;
                shooterMotor.setVelocity(ticksPerSec);
            }

            double actualRPM = (shooterMotor.getVelocity(AngleUnit.DEGREES) / 360.0) * 60.0;

            // ── Telemetry ──────────────────────────────────────────────────────
            telemetry.addData("Hood Servo", "");
            telemetry.addData("  Position",    "%.2f  (dpad up/down, step=%.2f)", servoPos, SERVO_STEP);
            telemetry.addData("  Range",       "[%.2f  →  %.2f]", SERVO_MIN, SERVO_MAX);
            telemetry.addLine();
            telemetry.addData("Shooter Motor", "");
            telemetry.addData("  Motor running",  motorRunning ? "YES  (RB to stop)" : "NO  (RB to start)");
            telemetry.addData("  Target RPM",  "%.0f  (dpad right/left, step=%.0f)", targetRPM, RPM_STEP);
            telemetry.addData("  Actual RPM",  "%.1f", actualRPM);
            telemetry.addData("  RPM error",   "%.1f", targetRPM - actualRPM);
            telemetry.update();

            lastDpadUp    = dpadUpNow;
            lastDpadDown  = dpadDownNow;
            lastDpadRight = dpadRightNow;
            lastDpadLeft  = dpadLeftNow;
            lastRB        = rbNow;
        }

        // Clean stop on OpMode end
        shooterMotor.setVelocity(0);
    }
}