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
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private Servo     hoodServo;
    private Servo     gateServo;

    // ── Tunable constants ─────────────────────────────────────────────────────
    // REV HD Hex Motor = 28 counts per revolution at the motor shaft (bare motor, no gearbox)
    // If you have a gearbox attached, multiply: e.g. 40:1 gearbox = 28 * 40 = 1120
    private static final double TICKS_PER_REV   = 28.0;   // ← FIXED (was 50, wrong for HD Hex)
    private static final double Y_MIN        = -72;
    private static final double Y_MAX        = 72;
    private static final double Y_STEP       = 6;
    private static final double X_STEP         = 6;
    private static final double X_MIN          = -72;
    private static final double X_MAX          = 72; // HD Hex free-spin ~6000 RPM

    // ── State ─────────────────────────────────────────────────────────────────
    private double  YPos      = 0.0;
    private double  XPos     = 0.0;
    private boolean motorRunning  = false;
    private boolean intakeRunning  = false;

    // Button edge-detection flags
    private boolean lastDpadUp    = false;
    private boolean lastDpadDown  = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft  = false;
    private boolean lastRB        = false;
    private boolean lastLB        = false;

    public static double velocity(double x, double y) {
        return (44.297 * Math.pow(Math.abs(y), 1.0 / 3.0)
                - 16.893 * Math.abs(x)
                + 0.0018285 * Math.pow(Math.abs(y), 2)
                + 5.0046e-05 * Math.pow(Math.abs(x), 2) * Math.pow(y, 2)
                + 1460); // * (TICKS_PER_REV / 60.0); // maybe the  * (TICKS_PER_REV / 60.0) not needed?
    }

    public static double hoodServo(double x, double y) {
        return 0.037524 * Math.pow(Math.abs(y), 0.5)
                - 0.027099 * Math.pow(Math.abs(x), 1.0 / 3.0)
                - 0.0051773 * Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))
                + 0.18998;
    }

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

        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMain");
        intakeMotor.setVelocity(0);
        transferMotor = hardwareMap.get(DcMotorEx.class, "IntakeControl");
        transferMotor.setVelocity(0);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setPosition(servoPos);
        gateServo = hardwareMap.get(Servo.class, "Gate");
        gateServo.setPosition(servoPos);

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
            boolean lbNow        = gamepad1.left_bumper;

            boolean pressedUp    = dpadUpNow    && !lastDpadUp;
            boolean pressedDown  = dpadDownNow  && !lastDpadDown;
            boolean pressedRight = dpadRightNow && !lastDpadRight;
            boolean pressedLeft  = dpadLeftNow  && !lastDpadLeft;
            boolean pressedRB    = rbNow        && !lastRB;
            boolean pressedLB    = lbNow        && !lastLB;

            // ── dpad up/down → servo position ─────────────────────────────────
            if (pressedUp) { YPos = Math.min(YPos + Y_STEP, Y_MAX); }
            if (pressedDown) { YPos = Math.max(YPos - Y_STEP, Y_MIN); }
            if (pressedRight) { XPos = Math.min(XPos + X_STEP, X_MAX); }
            if (pressedLeft) { XPos = Math.max(XPos - X_STEP, X_MIN); }

            if (pressedRB) {
                motorRunning = !motorRunning;
                if (!motorRunning) {
                    shooterMotor.setVelocity(0);
                    gateServo.setPosition(1);
                }
            }

            if (pressedLB) {
                intakeRunning = !intakeRunning;
                if (!intakeRunning) {
                    intakeMotor.setVelocity(0);
                    if (!gamepad1.A) {
                        transferMotor.setVelocity(0);
                    } else {
                        transferMotor.setPower(-1);
                    }
                }
            }z

            if (intakeRunning) {
                intakeMotor.setPower(1);
                transferMotor.setPower(1);
            }

            double targetTicksPerSec = velocity(XPos, YPos);
            double targetServo = hoodServo(XPos, YPos);
            if (motorRunning) {
                shooterMotor.setVelocity(targetTicksPerSec);
                hoodServo.setPosition(targetServo);
                gateServo.setPosition(0.8);
            }

            double rawVelTicksPerSec  = shooterMotor.getVelocity();

            double velDegPerSec = shooterMotor.getVelocity(AngleUnit.DEGREES);
            double actualRPM    = (velDegPerSec / 360.0) * 60.0;

            double motorPower   = shooterMotor.getPower();
            double motorCurrent = shooterMotor.getCurrent(CurrentUnit.AMPS);
            int    encoderTicks = shooterMotor.getCurrentPosition();
            double rpmError     = targetRPM - actualRPM;

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

            lastDpadUp    = dpadUpNow;
            lastDpadDown  = dpadDownNow;
            lastDpadRight = dpadRightNow;
            lastDpadLeft  = dpadLeftNow;
            lastRB        = rbNow;
        }

        shooterMotor.setVelocity(0);

    }
}
