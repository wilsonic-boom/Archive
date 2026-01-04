package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Motor Speed Test (Manual Distance)", group="Testing")
public class setup extends LinearOpMode {

    private DcMotorEx motorFL, motorFR, motorBL, motorBR;

    double[] speeds = {0.25, 0.50, 0.75, 1.0};
    int speedIndex = 0;
    boolean rotate = false;

    long intervalMs = 100;

    @Override
    public void runOpMode() {

        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");

        DcMotorEx[] motors = {motorFL, motorFR, motorBL, motorBR};

        // Initialize motors
        for (DcMotorEx m : motors) {
            m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("READY — Motor Speed Test");
        telemetry.addLine("Dpad Left/Right = change speed");
        telemetry.addLine("Dpad Up/Down = change duration (100ms steps)");
        telemetry.addLine("A = run test");
        telemetry.update();

        waitForStart();

        // Button edge detection variables
        boolean lastLeft = false, lastRight = false;
        boolean lastUp = false, lastDown = false;
        boolean lastA = false;

        while (opModeIsActive()) {

            // ====== BUTTON EDGE DETECTION ======
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean aPressed = gamepad1.a;

            if (gamepad1.b) {
                rotate = !rotate;
            }

            // Speed up/down (only on button press, not hold)
            if (left && !lastLeft) speedIndex = Math.max(0, speedIndex - 1);
            if (right && !lastRight) speedIndex = Math.min(speeds.length - 1, speedIndex + 1);

            // Duration up/down
            if (up && !lastUp) intervalMs += 100;
            if (down && !lastDown) intervalMs = Math.max(100, intervalMs - 100);

            // Save new "last" states
            lastLeft = left;
            lastRight = right;
            lastUp = up;
            lastDown = down;

            double power = speeds[speedIndex];

            telemetry.addData("Power", power);
            telemetry.addData("Interval (ms)", intervalMs);
            telemetry.addLine("Press A to run");
            telemetry.update();

            // ====== RUN TEST ======
            if (aPressed && !lastA) {

                // Reset encoders
                for (DcMotorEx m : motors) {
                    m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                }

                // Start motors
                for (DcMotorEx m : motors) {
                    m.setPower(power);
                }
                motorFL.setPower(power);
                motorFR.setPower(-power);
                motorBL.setPower(power);
                motorBR.setPower(-power);

                long testStart = System.currentTimeMillis();

                // Safer wait loop that still lets the OpMode stop gracefully
                while (opModeIsActive() && System.currentTimeMillis() - testStart < intervalMs) {
                    idle();
                }

                // Stop motors
                for (DcMotorEx m : motors) {
                    m.setPower(0);
                }

                // Collect encoder values
                int fl = motorFL.getCurrentPosition();
                int fr = motorFR.getCurrentPosition();
                int bl = motorBL.getCurrentPosition();
                int br = motorBR.getCurrentPosition();

                telemetry.addLine("\n==== TEST COMPLETE ====");
                telemetry.addData("Power", power);
                telemetry.addData("Duration (ms)", intervalMs);
                telemetry.addData("FL Encoder", fl);
                telemetry.addData("FR Encoder", fr);
                telemetry.addData("BL Encoder", bl);
                telemetry.addData("BR Encoder", br);
                telemetry.addLine("Measure distance with ruler now.");
                telemetry.update();

                sleep(1200);
            }

            lastA = aPressed;
        }
    }
}
