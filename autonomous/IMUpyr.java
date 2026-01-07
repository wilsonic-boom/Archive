package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "IMUpyr")
public class IMUpyr extends LinearOpMode {
    // This is the code that sets up the IMU, allowing us to get values from it.
    private IMU imu;

    @Override
    public void runOpMode() {

        // This is the orientation. AMEND THIS LATER WHEN WE KNOW THE PROPER MOUNTING.
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        while (opModeIsActive()) {
            // This Gets the orientation
            Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES);

            double yaw = angles.firstAngle;   // Z axis
            double pitch = angles.secondAngle; // Y axis
            double roll = angles.thirdAngle;   // X axis

            telemetry.addData("This Is The Yaw: ", yaw);
            telemetry.addData("This Is The Pitch", pitch);
            telemetry.addData("This Is The Roll", roll);
            telemetry.update();
        }
    }
}
