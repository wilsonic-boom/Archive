/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.reflect.Field;

@Autonomous(name="Drive with Motor encoders", group="Robot")

public class motor_encodersmotor extends LinearOpMode {

    enum Alliance { RED, BLUE }
    enum Start { TOP, BOTTOM }

    Alliance alliance;
    Start start;

    /* Declare OpMode members. */
    private DcMotor FLmotor    = null;
    private DcMotor FRmotor   = null;
    private DcMotor BLmotor  = null;
    private DcMotor BRmotor = null;;
    private DcMotor IntakeMain = null;;
    private DcMotor IntakeControl = null;

    private DcMotorEx Shooter;
    private Servo hoodServo;
    private Servo gateServo;

    private IMU imu;
    Orientation angles;

    double FieldX        = 0.0;  // inches
    double FieldY        = 0.0;  // inches
    double FieldRotation = 0.0;  // degrees


    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV  = 537.6;
    static final double DRIVE_GEAR_REDUCTION  = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED  = 0.6;
    static final double TURN_SPEED   = 0.5;


    @Override
    public void runOpMode() {

        // Set alliance and start position here
        start    = Start.BOTTOM;
        alliance = Alliance.BLUE;


        FLmotor     = hardwareMap.get(DcMotor.class, "FLmotor");
        FRmotor    = hardwareMap.get(DcMotor.class, "FRmotor");
        BLmotor  = hardwareMap.get(DcMotor.class, "BLmotor");
        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");

        IntakeMain  = hardwareMap.get(DcMotor.class, "IntakeMain");
        IntakeControl = hardwareMap.get(DcMotor.class, "IntakeControl");

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hoodServo = hardwareMap.get(Servo.class, "Hood");
        gateServo = hardwareMap.get(Servo.class, "Gate");


        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMain.setDirection(DcMotorSimple.Direction.REVERSE);


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);

        // Reset drive encoders
        FLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized — waiting for Start");
        telemetry.update();

        // START
        waitForStart();


        if (opModeIsActive()) {
            main_code();
        }

        // display final position until Stop is pressed
        while (opModeIsActive()) {
            telemetry.addData("Path", "Complete");
            telemetry.addData("Final Pos", "X: %.2f  Y: %.2f  Rot: %.2f",
                    FieldX, FieldY, FieldRotation);
            telemetry.update();
            sleep(100);
        }
    }


    // encoderDrive
    //   xInches      — lateral movement (positive = right)
    //   yInches      — forward/back movement (positive = forward)
    //   turnDegrees  — heading to face (field-centric) or delta turn (relative)
    //   fieldcentric — if true, x/y/turn are absolute field coordinates
    //   timeoutS     — max seconds before giving up
    public void encoderDrive(double xInches, double yInches,
                             double turnDegrees, boolean fieldcentric,
                             double timeoutS) {

        // 1. Strafe Multiplier: Compensates for mecanum slip during sideways movement
        double strafeMult = 1.15;
        double adjustedX = xInches * strafeMult;
        double adjustedY = yInches;
        double adjustedTurn = turnDegrees;

        if (fieldcentric) {
            // 2. Vector Rotation: This is the "Magic" that makes Field Centric work.
            // It rotates your X and Y inputs based on the robot's current heading.
            double robotRad = Math.toRadians(-FieldRotation);
            adjustedX = (xInches * strafeMult * Math.cos(robotRad)) - (yInches * Math.sin(robotRad));
            adjustedY = (xInches * strafeMult * Math.sin(robotRad)) + (yInches * Math.cos(robotRad));

            // Calculate how much we need to turn from where we are now
            adjustedTurn = turnDegrees - FieldRotation;
        }

        // Normalise turn to (-180, 180] to prevent "the long way around" turns
        adjustedTurn = (adjustedTurn + 180) % 360 - 180;

        // Update the "Memory" of where the robot is on the field
        FieldX += xInches;
        FieldY += yInches;
        FieldRotation += adjustedTurn;

        if (!opModeIsActive()) return;

        final double TRACK_WIDTH = 12.8;
        final double WHEEL_BASE  = 8.8;

        double turnRadians = Math.toRadians(adjustedTurn);
        double robotRadius = (TRACK_WIDTH / 2.0) + (WHEEL_BASE / 2.0);
        double turnInches  = robotRadius * turnRadians;

        // 3. Mecanum Mixing
        double TL_dist = adjustedY + adjustedX + turnInches;
        double TR_dist = adjustedY - adjustedX - turnInches;
        double BL_dist = adjustedY - adjustedX + turnInches;
        double BR_dist = adjustedY + adjustedX - turnInches;

        // Set Targets
        FLmotor.setTargetPosition(FLmotor.getCurrentPosition() + (int)(TL_dist * COUNTS_PER_INCH));
        FRmotor.setTargetPosition(FRmotor.getCurrentPosition() + (int)(TR_dist * COUNTS_PER_INCH));
        BLmotor.setTargetPosition(BLmotor.getCurrentPosition() + (int)(BL_dist * COUNTS_PER_INCH));
        BRmotor.setTargetPosition(BRmotor.getCurrentPosition() + (int)(BR_dist * COUNTS_PER_INCH));

        FLmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 4. Power Scaling: Ensures all motors finish at the exact same time
        double maxDist = Math.max(Math.max(Math.abs(TL_dist), Math.abs(TR_dist)),
                Math.max(Math.abs(BL_dist), Math.abs(BR_dist)));

        if (maxDist > 0) {
            FLmotor.setPower(Math.abs(TL_dist / maxDist) * DRIVE_SPEED);
            FRmotor.setPower(Math.abs(TR_dist / maxDist) * DRIVE_SPEED);
            BLmotor.setPower(Math.abs(BL_dist / maxDist) * DRIVE_SPEED);
            BRmotor.setPower(Math.abs(BR_dist / maxDist) * DRIVE_SPEED);
        }

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (FLmotor.isBusy() || FRmotor.isBusy() || BLmotor.isBusy() || BRmotor.isBusy())) {
            idle(); // Keep the loop alive
        }

        // Stop and Reset to Encoders
        FLmotor.setPower(0); FRmotor.setPower(0);
        BLmotor.setPower(0); BRmotor.setPower(0);
        FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public double[] FindPos() {
        return new double[]{ FieldX, FieldY, FieldRotation };
    }

    public void main_code() {
        int timeout = 10;
        int change  = 1;
        if (alliance == Alliance.BLUE) {
            change = -1;
        }

        double row_start = -36.0 + 7.5 + 9; // = -19.5

        if (start == Start.BOTTOM) {

            // ── Shoot from starting position ──
            // ADD SHOOTING HERE
            telemetry.addData("bottomblue","true");

            // ── Intake row 1 ──
            gateServo.setPosition(1);
            encoderDrive(row_start, -36, 270, true, timeout);
            IntakeMain.setPower(1);
            IntakeControl.setPower(1);
            encoderDrive(0, (72.0 - row_start), 0, false, 4);
            IntakeMain.setPower(0);
            IntakeControl.setPower(0);
            //shoot

            // ── Intake row 2 ──
            encoderDrive(row_start, -12, 270, true, timeout);
            IntakeMain.setPower(1);
            IntakeControl.setPower(1);
            encoderDrive(0, (72.0 - row_start), 0, false, timeout);
            IntakeMain.setPower(0);
            IntakeControl.setPower(0);

            // ── Open classifier ──
            encoderDrive(change * row_start, 0, 270, true, timeout); // CAN CHANGE X IF NOT NEEDED THAT FAR
            IntakeMain.setPower(1);
            IntakeControl.setPower(1);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);
            IntakeMain.setPower(0);
            IntakeControl.setPower(0);
            encoderDrive(change * -(72.0 - row_start), 0, 0, false, timeout);

            // ── Go to shoot pos and shoot ──
            encoderDrive(0, 12, 0, false, timeout);

            //shoot

            // ── Intake row 3 ──
            encoderDrive(change * row_start, 12, 270, true, timeout);
            IntakeMain.setPower(1);
            IntakeControl.setPower(1);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);
            IntakeMain.setPower(0);
            IntakeControl.setPower(0);
            // ── Go to shoot pos and shoot ──
            // ADD HERE GO TO POS
            // ADD HERE SHOOTING

        } else { // Start.TOP

            // ── Face april tag ──
            encoderDrive(change * -24, -24, 180, false, timeout);

            // ── Calibrate starting position and shoot ──
            // ADD FINDING START AND SHOOTING HERE

            // ── Intake row 3 ──
            encoderDrive(change * row_start, 12, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // ── Go to shoot pos and shoot ──
            // ADD GO TO SHOOTING POS AND SHOOT HERE

            // ── Open classifier ──
            encoderDrive(change * row_start, 0, 270, true, timeout); // CAN CHANGE X IF NOT NEEDED THAT FAR
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);
            encoderDrive(change * -(72.0 - row_start), 0, 0, false, timeout);

            // ── Intake row 2 ──
            encoderDrive(change * row_start, -12, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // ── Go to shoot pos and shoot ──
            encoderDrive(change * row_start, -36, 270, true, timeout);
            // ADD GO TO SHOOT POS AND SHOOT HERE

            // ── Intake row 1 ──
            encoderDrive(change * row_start, -36, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // ── Go to shoot pos and shoot ──
            // ADD GO TO SHOOT POS AND SHOOT HERE
        }
    }

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

    public static List<Double> close_shooter_pos(double x, double y) {
        List<Double> result = new ArrayList<>();

        if (y>x-48 && y<-x && y<x && y>-x-48) {
            if (y < -24) {
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
        if (!(y<x-48 && y<-x-48) && !(-x<y && x<y)) { // if its not alr in shooting area, go to it and face shooter
            List<Double> values = close_shooter_pos(x, y);

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

    public void goAndShoot() {
        goToArea(FieldX, FieldY); // go to shooter area and face shooter
        Shooter.setVelocity(targetTicksPerSec);
        hoodServo.setPosition(targetServo);
        sleep(750);
        gateServo.setPosition(0.8);
        IntakeControl.setPower(-1);
        sleep(500);
        IntakeControl.setVelocity(0);
        gateServo.setPosition(1);
        sleep(100);
    }

}
