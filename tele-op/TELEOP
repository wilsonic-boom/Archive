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

@TeleOp
public class Teleop extends LinearOpMode {


    boolean Slow = false;

    boolean reverse = false;

    private DcMotorEx Shooter = null;


    boolean intake = false;

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
                FLmotor.setPower(FLpower/2);
                BLmotor.setPower(BLpower/2);
                FRmotor.setPower(FRpower/2);
                BRmotor.setPower(BRpower/2);
                telemetry.addData("Speed", "Slow");
            }



            // SHOOTER

            Shooter.setPower(gamepad1.right_trigger);
            telemetry.addData("SHOOTER", Shooter.getVelocity());




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


            if (gamepad1.rightBumperWasPressed()) {
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

            telemetry.addData("GATE", gate.getPosition());
            telemetry.update();


        }
    }
}

