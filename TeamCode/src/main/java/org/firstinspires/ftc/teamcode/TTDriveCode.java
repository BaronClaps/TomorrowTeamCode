package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TTDriveCode", group="Linear Opmode")
public class TTDriveCode extends LinearOpMode {

    //---------------Declare Hardware Variables-----------------------//
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private Servo airplane = null;
    private Servo armROT = null;
    private Servo clawleft = null;
    private Servo clawright = null;
    private Servo clawrotate = null;

    //---------------Declare Variables-----------------------//
    private int bspeed;
    private double rfspeed;
    private double lfspeed;
    private double rbspeed;
    private double lbspeed;
    private double armR;
    //---------------Declare Servo Variables-----------------//
    double ClosedLeft = 0;
    double ClosedRight = 0.2;
    double ScoringClaw = 0.5;
    double ScoringArm = 0.18;
    double OpenLeft = 0.2;
    double OpenRight = 0;
    double GroundClaw = 0.4;
    double GroundArm = 0.11; //0.975;

    //---------------Run OpMode-----------------------------//
    @Override
    public void runOpMode() {
        //---------------Init Hardware-----------------------//
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        arm = hardwareMap.get(DcMotor.class, "arm");
        airplane = hardwareMap.get(Servo.class, "airplane");
        armROT = hardwareMap.get(Servo.class,"armROT");
        clawleft = hardwareMap.get(Servo.class, "clawleft");
        clawright = hardwareMap.get(Servo.class, "clawright");
        clawrotate = hardwareMap.get(Servo.class, "clawrotate");
        //---------------Setup Motors-----------------------//
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        //---------------Setup Servos-----------------------//
        armR = GroundArm;
        clawleft.setPosition(OpenLeft);
        clawright.setPosition(OpenRight);
        clawrotate.setPosition(GroundClaw);
        bspeed = 2;
        //---------------Wait until Play-----------------------//
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        //---------------Loop while Active-----------------------//
        while (opModeIsActive()) {

            //--------Joysticks Controls & Wheel Power-----------//
            double max;
            double axial   = -gamepad1.left_stick_y;  //Pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0)
            {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            //----------Arm-----------//

            if(gamepad2.b)
            {
                arm.setPower(1);
            }
            else
            {
                arm.setPower(0);
            }

            if(gamepad2.a)
            {
                arm.setPower(-1);
            }
            else
            {
                arm.setPower(0);
            }

            //------Arm Rotate--------//
            armROT.setPosition(armR);

            if (gamepad2.left_bumper){
                armR = GroundArm;
            }

            if (gamepad2.right_bumper){
                armR = ScoringArm;
            }

            //---------Airplane----------//

            if(gamepad1.y)
            {
                airplane.setPosition(1);
            }
            else
            {
                airplane.setPosition(0.5);
            }

            //----------Claws & Claw Rotate----------//

            if(gamepad2.right_trigger > 0.5)
            {
                clawright.setPosition(ClosedRight);
                clawleft.setPosition(ClosedLeft);
            }

            if(gamepad2.left_trigger > 0.5)
            {
                clawright.setPosition(OpenRight);
                clawleft.setPosition(OpenLeft);
            }

            if(gamepad2.x)
            {
                clawrotate.setPosition(GroundClaw);
            }

            if(gamepad2.y)
            {
                clawrotate.setPosition(ScoringArm);

            }

            //--------------Arm-Presets---------------//
            if(gamepad2.right_stick_button)
            {
                clawright.setPosition(OpenRight);
                clawleft.setPosition(OpenLeft);
                clawrotate.setPosition(GroundClaw);
                sleep(100);
                armR = GroundArm;

            }

            if(gamepad2.left_stick_button)
            {
                clawright.setPosition(ClosedRight);
                clawleft.setPosition(ClosedLeft);
                clawrotate.setPosition(ScoringClaw);
                sleep(100);
                armR = ScoringArm;

            }

            //-----------Speed Control------------//

            if(gamepad1.left_bumper)
            {
                bspeed = 1;
            }

            if(gamepad1.right_bumper)
            {
                bspeed = 2;
            }

            if(bspeed == 1)
            {
                lfspeed = leftFrontPower/2;
                rfspeed = rightFrontPower/2;
                lbspeed = leftBackPower/2;
                rbspeed = rightBackPower/2;
            }

            if(bspeed == 2)
            {
                lfspeed = leftFrontPower;
                rfspeed = rightFrontPower;
                lbspeed = leftBackPower;
                rbspeed = rightBackPower;
            }

            leftFrontDrive.setPower(lfspeed);
            rightFrontDrive.setPower(rfspeed);
            leftBackDrive.setPower(lbspeed);
            rightBackDrive.setPower(rbspeed);

            //-------------Display Timer & Wheel Power---------------//
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

        }}}