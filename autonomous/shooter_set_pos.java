package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "shooter_set_pos", group = "Linear OpMode")
public class shooter_set_pos extends LinearOpMode {

    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private Servo hoodServo;
    private Servo gateServo;

    // ── Tunable constants ─────────────────────────────────────────────────────
    // REV HD Hex Motor = 28 counts per revolution at the motor shaft (bare motor, no gearbox)
    // If you have a gearbox attached, multiply: e.g. 40:1 gearbox = 28 * 40 = 1120
    private static final double TICKS_PER_REV = 28.0;   // ← FIXED (was 50, wrong for HD Hex)
    private static final double Y_MIN = -72;
    private static final double Y_MAX = 72;
    private static final double Y_STEP = 6;
    private static final double X_STEP = 6;
    private static final double X_MIN = -72;
    private static final double X_MAX = 72; // HD Hex free-spin ~6000 RPM

    // ── State ─────────────────────────────────────────────────────────────────
    private double YPos = 0.0;
    private double XPos = 0.0;
    private double bearing = 0.0;
    private boolean motorRunning  = false;
    private boolean intakeRunning  = false;
    static boolean red = true;

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

    public static double hoodServoCalc(double x, double y) {
        return 0.037524 * Math.pow(Math.abs(y), 0.5)
                - 0.027099 * Math.pow(Math.abs(x), 1.0 / 3.0)
                - 0.0051773 * Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))
                + 0.18998;
    }

    public static double distance_formula(double x1, double y1, double x2, double y2) {
        return Math.sqrt(
                Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2)
        );
    }

    public static List<Integer> close_shooter_pos(double x, double y) {
        List<Integer> result = new ArrayList<>();

        if (y>x-48 && y<-x && y<x && y<-x-48) {
            if y < -24 {
                result.add(0);
                result.add(-48);
            } else {
                result.add(0);
                result.add(0);
            }
        } else if (x<0) {
            double x1 = (x-y)/2;
            double y1 = (y-x)/2;
            double x2 = (x+y+48)/2;
            double y2 = (x+y-48)/2;

            if (distance_formula(x, y, x1, y1) < distance_formula(x, y, x2, y2)) {
                result.add(x1);
                result.add(y1);
            } else {
                result.add(x2);
                result.add(y2);
            }
        } else {
            double x1 = (x+y)/2;
            double y1 = x1;
            double x2 = (x-y-48)/2;
            double y2 = (y-x-48)/2;

            if (distance_formula(x, y, x1, y1) < distance_formula(x, y, x2, y2)) {
                result.add(x1);
                result.add(y1);
            } else {
                result.add(x2);
                result.add(y2);
            }
        }

        return result;
    }

    public static void goToArea(double x, double y) {
        if (!(y<x-48 && y<-x-48) || !(-x<y && x<y)) { // if its not alr in shooting area, go to it and face shooter
            List<Integer> values = close_shooter_pos(x, y);

            double x1 = values.get(0);
            double y1 = values.get(1);
            double x2 = 0;
            double y2 = 66;

            if (red) {
                x2 = 66;
            } else {
                x2 = -66;
            }
            double bearing = (90 - Math.toDegrees(Math.atan2(y2 - y1, x2 - x1)) + 360) % 360;

            encoderdrive(x1, y1, bearing, true, 5);
        } else { // face shooter
            double x2 = 0;
            double y2 = 66;

            if (red) {
                x2 = 66;
            } else {
                x2 = -66;
            }
            double bearing = (90 - Math.toDegrees(Math.atan2(y2 - y, x2 - x)) + 360) % 360;

            encoderdrive(x, y, bearing, true, 5);
        }
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

        hoodServo = hardwareMap.get(Servo.class, "Hood");
        gateServo = hardwareMap.get(Servo.class, "Gate");

        // ── Init telemetry ─────────────────────────────────────────────────────
        telemetry.addData("Status",        "Initialized — waiting for start");
        telemetry.addData("TICKS_PER_REV", TICKS_PER_REV);
        telemetry.addData("Motor mode",    shooterMotor.getMode());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean dpadUpNow    = gamepad1.dpad_up;
            boolean dpadDownNow  = gamepad1.dpad_down;
            boolean dpadRightNow = gamepad1.dpad_right;
            boolean dpadLeftNow  = gamepad1.dpad_left;

            boolean pressedUp    = dpadUpNow    && !lastDpadUp;
            boolean pressedDown  = dpadDownNow  && !lastDpadDown;
            boolean pressedRight = dpadRightNow && !lastDpadRight;
            boolean pressedLeft  = dpadLeftNow  && !lastDpadLeft;

            if (pressedUp) { YPos = Math.min(YPos + Y_STEP, Y_MAX); }
            if (pressedDown) { YPos = Math.max(YPos - Y_STEP, Y_MIN); }
            if (pressedRight) { XPos = Math.min(XPos + X_STEP, X_MAX); }
            if (pressedLeft) { XPos = Math.max(XPos - X_STEP, X_MIN); }

            double targetTicksPerSec = velocity(XPos, YPos);
            double targetServo = hoodServoCalc(XPos, YPos);

            if (gamepad1.right_bumper) {
                goToArea(); // go to shooter area and face shooter
                shooterMotor.setVelocity(targetTicksPerSec);
                hoodServo.setPosition(targetServo);
                sleep(00);
                gateServo.setPosition(0.8);
                transferMotor.setPower(-1);
                sleep(500);
                transferMotor.setVelocity(0);
                gateServo.setPosition(1);
                sleep(100);
            } else {
                shooterMotor.setVelocity(0);
                gateServo.setPosition(1);
            }

            if (gamepad1.left_bumper) {
                intakeMotor.setPower(-1);
                transferMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
            }

            double rawVelTicksPerSec  = shooterMotor.getVelocity();

            double velDegPerSec = shooterMotor.getVelocity(AngleUnit.DEGREES);
            double actualRPM    = (velDegPerSec / 360.0) * 60.0;

            double motorPower   = shooterMotor.getPower();
            double motorCurrent = shooterMotor.getCurrent(CurrentUnit.AMPS);
            int    encoderTicks = shooterMotor.getCurrentPosition();

            telemetry.addLine("Position");
            telemetry.addData("  X",   "%.3f  (dpad L/R)", XPos);
            telemetry.addData("  Y",   "%.3f  (dpad up/down)", YPos);

            telemetry.addLine("=== HOOD SERVO ===");
            telemetry.addData("  Position",   "%.3f  (automatically done)", hoodServo.getPosition());

            telemetry.addLine();
            telemetry.addLine("=== SHOOTER MOTOR ===");
            telemetry.addData("  State",         motorRunning ? "RUNNING (RB=stop)" : "STOPPED (RB=start)");
            telemetry.addData("  RunMode",        shooterMotor.getMode());

            telemetry.addLine();
            telemetry.addLine("-- Velocity --");
            telemetry.addData("  Target ticks/sec",    "%.1f", targetTicksPerSec);
            telemetry.addData("  Actual ticks/sec",    "%.1f", rawVelTicksPerSec);
            telemetry.update();

            lastDpadUp    = dpadUpNow;
            lastDpadDown  = dpadDownNow;
            lastDpadRight = dpadRightNow;
            lastDpadLeft  = dpadLeftNow;
        }

        shooterMotor.setVelocity(0);

    }
}