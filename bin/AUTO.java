package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.opencv.core.Mat;

import java.lang.Math;


@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class AUTO extends LinearOpMode {

    // Declare OpMode members.
    private DcMotorEx motorFL = null;
    private DcMotorEx motorFR = null;
    private DcMotorEx motorBL = null;
    private DcMotorEx motorBR = null;
    private DcMotorEx motorST = null;
    private Thread Shoot = null;
    //    private IMU imu;

    // MATH Setup
    double x = 0;
    double y = 0;
    double ex = 0;
    double ey = 0;
    double orientation = 0;
    double eorientation = 0;
    int side = -1;

    public void setOrientation(double deg){
        double L = 30/2.5;
        double W = 40/2.5;
        double radius = 2;
        double RPM = 200;
        if (deg > orientation) {
            double ddeg = deg - orientation;
            double t = (ddeg * (L + W) * 60 * 1000) / (2 * Math.PI * radius * RPM);

            //𝑟 r = wheel radius
            //𝑅 R = motor RPM
            //𝐿 L = half the robot’s length (distance from center to front/back axle)
            //𝑊 W = half the robot’s width (distance from center to left/right axle)
            //=2r = wheel diameter
            //𝜃 θ = desired rotation angle (in radians)
            //𝑡 t = time to power the motors

            motorFL.setPower(-1);
            motorFR.setPower(-1);
            motorBL.setPower(-1);
            motorBR.setPower(-1);
            sleep((long) t);
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);

            orientation = deg;
        } else {
            double ddeg = orientation - deg;
            double t = (ddeg * (L + W) * 60 * 1000) / (2 * Math.PI * radius * RPM);

            motorFL.setPower(1);
            motorFR.setPower(1);
            motorBL.setPower(1);
            motorBR.setPower(1);
            sleep((long) t);
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }
    }

    public void moveDistance(double d){
        int diameter = 4;
        int RPM = 200;
        double t = (60 * d * 1000) / (Math.PI * diameter * RPM);
        motorFL.setPower(1);
        motorFR.setPower(1);
        motorBL.setPower(1);
        motorBR.setPower(1);
        sleep((long) t);
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    public void moveTo(double nx, double ny, double nr){
        int hi = 0;
        double distance = Math.pow( Math.pow(x - nx, 2) + Math.pow(y - ny, 2), 0.5);
        double angle = 0;
        if ((x <= nx) && (y <= ny)) {
            angle = Math.toDegrees( Math.atan((nx-x) / (ny-y)) );
        } else if ( (nx <= x) && (y <= ny) ) {
            angle = 360 - Math.toDegrees( Math.atan((x-nx) / (ny-y)) );
        } else if (x <= nx) { // ny <= y will always be true here, no need to check
            angle = 180 - Math.toDegrees( Math.atan((nx-x) / (y-ny)) );
        } else {
            angle = 180 + Math.toDegrees( Math.atan((x-nx) / (y-ny)) );
        }

        setOrientation(angle);
        moveDistance(distance);

        x = nx;
        y = ny;
    }

    public void goShootingArea(){
        // go to closest shooting area place
        // y = x m = -1
        // y = -x m = 1
        // y = x-48 m = -1
        // y = -x-48 m = 1
        if ((y > -x - 48) && (y > x - 48) && (y < -x) && (y < x)) {
            moveTo(0, 20, 0);
        } else {
            //a = m*b - x + y;
            //m*b - x + y = b;
            //bm*b - b = (x-y)/(m-1);
            if (x > 0){
                // m = -1
                // y = x
                double x1 = (x-y)/(-2);
                double y1 = x1;
                // y = x - 48
                double x2 = (x+y+48)/(-2);
                double y2 = x2 - 48;

                double r1 = Math.pow( Math.pow(x - x1, 2) + Math.pow(y - y1, 2), 0.5);
                double r2 = Math.pow( Math.pow(x - x2, 2) + Math.pow(y - y2, 2), 0.5);

                if (r1 > r2) {
                    moveTo(x1, y1, 0);
                } else {
                    moveTo(x2, y2, 0);
                }
            } else {
                // m = 1
                // y = -x
                double x1 = (x+y)/(2);
                double y1 = -x1;
                // y = -x - 48
                double x2 = (-x+y+48)/(-2);
                double y2 = -x2 - 48;

                double r1 = Math.pow( Math.pow(x - x1, 2) + Math.pow(y - y1, 2), 0.5);
                double r2 = Math.pow( Math.pow(x - x2, 2) + Math.pow(y - y2, 2), 0.5);

                if (r1 > r2) {
                    moveTo(x1, y1, 0);
                } else {
                    moveTo(x2, y2, 0);
                }
            }

        }
    }

    public void autoShoot(){
        // check if it is in the shooting area
        if ( !(((x < y) && (-x < y)) || ((y < x - 48) && (y < -x - 48))) ) {
            goShootingArea();
        }

        double goalx = 72 * side;
        double goaly = 72;

        double deg = (360 + side * Math.toDegrees(Math.acos((72 - y)/(Math.pow( Math.pow(goalx - x, 2) + Math.pow(goaly - y, 2), 0.5))))) % 360;

        setOrientation((double) deg);
        motorST.setPower(1);
    }

    public void updateOdometry(double a, double b, double c, double d, double t) {
        double circumference = Math.PI * 4;

        // convert rpm to meters travelled in dt
        double dFL = circumference * (a / 60.0) * t;
        double dFR = circumference * (b / 60.0) * t;
        double dBL = circumference * (c / 60.0) * t;
        double dBR = circumference * (d / 60.0) * t;

        double L = 30/2.5;
        double W = 40/2.5;
        // mecanum
        double dx_r = (dFL + dFR + dBL + dBR) / 4.0;
        double dy_r = (-dFL + dFR + dBL - dBR) / 4.0;
        double dTheta = (-dFL + dFR - dBL + dBR) / (4.0 * (L + W));

        double dx_w = dx_r * Math.cos(eorientation) - dy_r * Math.sin(eorientation);
        double dy_w = dx_r * Math.sin(eorientation) + dy_r * Math.cos(eorientation);

        ex += dx_w;
        ey += dy_w;
        eorientation += dTheta;
    }
    @Override
    public void runOpMode() {
        // INITIALIZE
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorST = hardwareMap.get(DcMotorEx.class, "motorST");
        // imu = hardwareMap.get(IMU.class, "imu");

        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);

        // MATH Setup
        x = -15;
        y = -27;
        orientation = 0;
        ex = -15;
        ey = -27;
        eorientation = 0;
        int test = 100;

        // start positions
        // startPos1 = (-15, -27, 0) or (15, -27, 0)
        // startPos2 = (-15, -27, 135) or (15, -27, 225)

        waitForStart();

        final int FPS = 60;
        final double FRAME_TIME = 1_000_000_000 / FPS; // in nanoseconds

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double start = System.nanoTime();

            // Send calculated power to wheels
            // Show the elapsed game time and wheel power.
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double powerFL = (y + x + rx) / denominator;
            double powerBL = (y - x + rx) / denominator;
            double powerFR = (y - x - rx) / denominator;
            double powerBR = (y + x - rx) / denominator;

            motorFL.setPower(powerFL);
            motorBL.setPower(powerBL);
            motorFR.setPower(powerFR);
            motorBR.setPower(powerBR);
            // Setup a variable for each drive wheel to save power level for telemetry
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            updateOdometry(motorFL.getVelocity(), motorFR.getVelocity(), motorBL.getVelocity(), motorBR.getVelocity(), 1/FPS);
            // Send calculated power to wheels
            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "power (%.2f)", y);
            telemetry.addData("Motors", "rpm (%.2f)", motorFR.getVelocity());
            telemetry.addData("Motors", "rpm (%.2f)", motorFL.getVelocity());
            telemetry.addData("Motors", "rpm (%.2f)", motorBR.getVelocity());
            telemetry.addData("Motors", "rpm (%.2f)", motorBL.getVelocity());

            telemetry.addData("Wanted Pos", "X: " + x + " Y: " + y + " Orientation: " + orientation);
            telemetry.addData("Estimated Pos", "X: " + ex + " Y: " + ey + " Orientation: " + eorientation);

            if (Shoot != null) {
                if (!Shoot.isAlive()) {
                    Shoot = null;
                    telemetry.addData("Travelling to shooting area", false);
                } else {
                    telemetry.addData("Travelling to shooting area", true);
                }
            }
            telemetry.update();

            if (gamepad1.leftBumperWasPressed()) {
                Shoot = new Thread(this::autoShoot);
                Shoot.start();
            }
            if (gamepad1.xWasPressed()) {
                Shoot.interrupt();
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBL.setPower(0);
                motorBR.setPower(0);
                sleep(test);
            }


            if (gamepad1.aWasPressed()) {
                motorFL.setPower(1);
                motorFR.setPower(1);
                motorBL.setPower(1);
                motorBR.setPower(1);
                sleep(test);
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBL.setPower(0);
                motorBR.setPower(0);
                test *= 2;
            }

            double elapsed = System.nanoTime() - start;
            double wait = (FRAME_TIME - elapsed) / 1000000;
            if (wait > 0) {
                sleep((long) wait);
            }
        }
    }
}
