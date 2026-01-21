package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="ThreeDeadwheel", group="Autonomous")
public class ThreeDeadwheel extends LinearOpMode {

    DcMotorEx lEnc, rEnc, sEnc; // Configurs the left right and centre encoders pos on the field and heading
    double x=0, y=0, h=0;
    int pl=0, pr=0, ps=0;

    @Override
    public void runOpMode() {

        lEnc = hardwareMap.get(DcMotorEx.class,"left");
        rEnc = hardwareMap.get(DcMotorEx.class,"right");
        sEnc = hardwareMap.get(DcMotorEx.class,"side");

        lEnc.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rEnc.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sEnc.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lEnc.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rEnc.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sEnc.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()){

            int l = lEnc.getCurrentPosition(); // Reads current encoder ticks
            int r = rEnc.getCurrentPosition();
            int s = sEnc.getCurrentPosition();

            double dL = (l-pl)*null; // these are the number of ticks, we must change this later to a value we know
            double dR = (r-pr)*null; // How much has the robot moved since the last llop?
            double dS = (s-ps)*null;

            pl = l; pr = r; ps = s; // Update prev values

            double dH = (dR-dL)/null;  // put real value later for distance between wheels
            double f = (dL+dR)/2; // forward movement
            double str = dS - dH*null;  // distance between centre and perpendicular wheel to calc strafe distance

            x += f*Math.cos(h)-str*Math.sin(h);
            y += f*Math.sin(h)+str*Math.cos(h);
            h += dH;

            telemetry.addData("X",x);
            telemetry.addData("Y",y);
            telemetry.addData("H deg",Math.toDegrees(h));
            telemetry.update();
        }
    }
}

