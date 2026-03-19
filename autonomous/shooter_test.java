package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "shooter_test", group = "Linear OpMode")
public class shooter_test extends LinearOpMode {

    private DcMotorEx shooterMotor;
    private Servo     hoodServo;

    // ── Tunable constants ─────────────────────────────────────────────────────
    // REV HD Hex Motor = 28 counts per revolution at the motor shaft (bare motor, no gearbox)
    // If you have a gearbox attached, multiply: e.g. 40:1 gearbox = 28 * 40 = 1120
    private static final double TICKS_PER_REV   = 28.0;   // ← FIXED (was 50, wrong for HD Hex)
    private static final double SERVO_MIN        = 0.0;
    private static final double SERVO_MAX        = 0.22;
    private static final double SERVO_STEP       = 0.01;
    private static final double RPM_STEP         = 100.0;
    private static final double RPM_MIN          = 0.0;
    private static final double RPM_MAX          = 6000.0; // HD Hex free-spin ~6000 RPM

    // ── State ─────────────────────────────────────────────────────────────────
    private double  servoPos      = 0.0;
    private double  targetRPM     = 0.0;
    private boolean motorRunning  = false;

    // Button edge-detection flags
    private boolean lastDpadUp    = false;
    private boolean lastDpadDown  = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft  = false;
    private boolean lastRB        = false;

    @Override
    public void runOpMode() {

        // ── Hardware init ──────────────────────────────────────────────────────
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        // FLOAT so the flywheel can spin down freely instead of braking hard
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setVelocity(0);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setPosition(servoPos);

        // ── Init telemetry ─────────────────────────────────────────────────────
        telemetry.addData("Status",        "Initialized — waiting for start");
        telemetry.addData("TICKS_PER_REV", TICKS_PER_REV);
        telemetry.addData("Motor mode",    shooterMotor.getMode());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ── Read gamepad ──────────────────────────────────────────────────
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

            // ── Apply velocity setpoint ────────────────────────────────────────
            // setVelocity(ticks/sec) — no AngleUnit = ticks per second
            double targetTicksPerSec = 0;
            if (motorRunning) {
                targetTicksPerSec = (targetRPM * TICKS_PER_REV) / 60.0;
                shooterMotor.setVelocity(targetTicksPerSec);
            }

            // ── Read back actual state ────────────────────────────────────────
            // getVelocity() with no args → ticks/sec (matches our setVelocity units)
            double rawVelTicksPerSec  = shooterMotor.getVelocity();

            // getVelocity(DEGREES) → degrees/sec, useful for an RPM display
            double velDegPerSec = shooterMotor.getVelocity(AngleUnit.DEGREES);
            double actualRPM    = (velDegPerSec / 360.0) * 60.0;

            double motorPower   = shooterMotor.getPower();
            double motorCurrent = shooterMotor.getCurrent(CurrentUnit.AMPS);
            int    encoderTicks = shooterMotor.getCurrentPosition();
            double rpmError     = targetRPM - actualRPM;

            // ── Telemetry ──────────────────────────────────────────────────────
            telemetry.addLine("=== HOOD SERVO ===");
            telemetry.addData("  Position",   "%.3f  (dpad up/down)", servoPos);
            telemetry.addData("  Range",      "[%.2f → %.2f]  step=%.2f",
                              SERVO_MIN, SERVO_MAX, SERVO_STEP);

            telemetry.addLine();
            telemetry.addLine("=== SHOOTER MOTOR ===");
            telemetry.addData("  State",         motorRunning ? "RUNNING (RB=stop)" : "STOPPED (RB=start)");
            telemetry.addData("  RunMode",        shooterMotor.getMode());

            telemetry.addLine();
            telemetry.addLine("-- Velocity --");
            telemetry.addData("  Target RPM",          "%.0f  (dpad L/R, step=%.0f)",
                              targetRPM, RPM_STEP);
            telemetry.addData("  Actual RPM",          "%.1f", actualRPM);
            telemetry.addData("  RPM error",           "%.1f  (%s)", rpmError,
                              Math.abs(rpmError) < 50 ? "ON TARGET" : "OFF");
            telemetry.addData("  Target ticks/sec",    "%.1f", targetTicksPerSec);
            telemetry.addData("  Actual ticks/sec",    "%.1f", rawVelTicksPerSec);

            telemetry.addLine();
            telemetry.addLine("-- Motor diagnostics --");
            telemetry.addData("  Output power",   "%.3f  (should rise toward 1.0)", motorPower);
            telemetry.addData("  Current (A)",    "%.2f  (stall ~9 A)", motorCurrent);
            telemetry.addData("  Encoder pos",    encoderTicks);

            telemetry.addLine();
            telemetry.addLine("-- Config check --");
            telemetry.addData("  TICKS_PER_REV", "%.0f  (HD Hex bare=28)", TICKS_PER_REV);
            telemetry.addData("  Max RPM set",   "%.0f", RPM_MAX);

            telemetry.update();

            // ── Update edge-detection flags ───────────────────────────────────
            lastDpadUp    = dpadUpNow;
            lastDpadDown  = dpadDownNow;
            lastDpadRight = dpadRightNow;
            lastDpadLeft  = dpadLeftNow;
            lastRB        = rbNow;
        }

        // Clean stop
        shooterMotor.setVelocity(0);
    }
}
