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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Arrays;


@TeleOp(name="completecodev6", group="Linear OpMode")
public class completecodev4 extends LinearOpMode {

    // Declare OpMode members.
    Servo sorter;
    Servo flap;
    private DcMotorEx motorFL = null;
    private DcMotorEx motorFR = null;
    private DcMotorEx motorBL = null;
    private DcMotorEx motorBR = null;
    private DcMotorEx shooter = null;
    private DcMotorEx intake = null;

    boolean aPressedLast = false;
    boolean flapIsOpen = false;
    boolean lbPressedLast = false;
    boolean rbPressedLast = false;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        intake  = hardwareMap.get(DcMotorEx.class, "intake");
        sorter  = hardwareMap.get(Servo.class, "sorter");
        flap  = hardwareMap.get(Servo.class, "flap");
        motorFL  = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR  = hardwareMap.get(DcMotorEx.class, "motorFR");
        motorBL  = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR  = hardwareMap.get(DcMotorEx.class, "motorBR");

        shooter  = hardwareMap.get(DcMotorEx.class, "shooter");

        // MOTIF
        ArrayList<ArrayList<String>> motif = new ArrayList<>();
        motif.add(new ArrayList<>(Arrays.asList("R", "R", "G")));
        motif.add(new ArrayList<>(Arrays.asList("R", "R", "G")));
        // ORDER
        ArrayList<String> order = new ArrayList<>(Arrays.asList("G", "R", "R"));
        // SHOOT
        shoot(motif, order);


        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        sorter.setPosition(1);
        flap.setPosition(1);

        waitForStart();
        while (opModeIsActive()) {

            // DRIVETRAIN
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

            // SHOOTER
            shooter.setPower(gamepad1.right_trigger);

            // FLAP AND SORTER
            double flapposition = flap.getPosition();
            double sorterposition = sorter.getPosition();

            boolean aPressedNow = gamepad1.a;

            boolean lbPressedNow = gamepad1.left_bumper;
            boolean rbPressedNow = gamepad1.right_bumper;


            if (gamepad1.dpad_right) {
                flap.setPosition(0);
            } else if (gamepad1.dpad_left) {
                flap.setPosition(1);
            }



            if (aPressedNow && !aPressedLast) { // flip the state
                if (flapIsOpen) {
                    flap.setPosition(0.7);
                } else {
                    flap.setPosition(1);
                }
                flapIsOpen = !flapIsOpen;
            }
            aPressedLast = aPressedNow;

            if (gamepad1.x) {
                sorter.setPosition((sorterposition + (double) 2 /3)%1);
            }

            if (lbPressedNow && !lbPressedLast) {
                //sorter.setPosition((0.2 + sorterposition)%4);
                sorter.setPosition((sorterposition - (double) 1 /3));
            }
            lbPressedLast = lbPressedNow;

            if (rbPressedNow && !rbPressedLast) {
                sorter.setPosition((sorterposition + (double) 1 /3));
            }
            rbPressedLast = rbPressedNow;

            // INTAKE

            if (gamepad1.dpad_down) {
                intake.setPower(0);
            } else if (gamepad1.dpad_up) {
                intake.setPower(1);
            } else {
                intake.setPower(0.5);
            }
            //telemetry.addData("Motors", "power (%.2f)", y);
            //telemetry.addData("Motors", "rpm (%.2f)", motorFR.getVelocity());
            //telemetry.addData("Motors", "rpm (%.2f)", motorFL.getVelocity());
            //telemetry.addData("Motors", "rpm (%.2f)", motorBR.getVelocity());
            //telemetry.addData("Motors", "rpm (%.2f)", motorBL.getVelocity());

            telemetry.addData("Shooter", "rpm (%.2f)", motorBL.getVelocity());
            telemetry.addData("Sorter", sorterposition);
            telemetry.addData("a", gamepad1.a);
            telemetry.addData("Flap", flapposition);
            telemetry.addData("aPressedLast", aPressedLast);
            telemetry.addData("aPressedNow", aPressedNow);
            telemetry.update();
        }
    }
    public void shoot(ArrayList<ArrayList<String>> motif, ArrayList<String> order) {

        while (!order.isEmpty()) {

            // rotate inside this function (no separate rotate())
            while (!order.get(0).equals(motif.get(1).get(0))) {
                // rotation logic here
                order.add(0, order.remove(order.size() - 1));
                // ROTATE THE BALLS HERE:
                // not done yet
            }
            // SHOOT HERE:
            // not done yet

            // pop values
            order.remove(0);
            motif.get(1).remove(0);
        }

        // motif.pop(1); motif.append(motif[0]) RESETTING MOTIF 1
        ArrayList<String> first = motif.get(0);
        motif.remove(1);
        motif.add(first);
    }


}
