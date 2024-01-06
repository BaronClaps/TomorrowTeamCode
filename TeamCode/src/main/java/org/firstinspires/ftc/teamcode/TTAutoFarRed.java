/* Copyright (c) 2023 FIRST. All rights reserved.
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

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous(name="TTAutoFarRed")

public class TTAutoFarRed extends LinearOpMode{

    private final int READ_PERIOD = 1;


    public int pixelspot = 2;


    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel
    private DcMotor arm = null;
    private Servo armROT     = null; //e1

    private DcMotor hang  = null;  //e3

    private Servo clawrotate = null; //es1

    private Servo airplane = null; //es3
    private Servo clawleft  = null; //es1
    private Servo clawright = null; //es2
    private Servo hangservo = null; //es4
    double ClosedLeft = 0;
    double ClosedRight = 0.2;
    double ScoringClaw = 0.5;
    double ScoringArm = 0.16;
    double OpenLeft = 0.2;
    double OpenRight = 0;
    double GroundClaw = 0.4;
    double GroundArm = 0.11; //0.975;



    private HuskyLens huskyLens;
    //TODO add your other motors and sensors here


    @Override public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        arm = hardwareMap.get(DcMotor.class, "arm");
        armROT = hardwareMap.get(Servo.class, "armROT");
        hang      = hardwareMap.get(DcMotor.class, "hang");
        clawrotate = hardwareMap.get(Servo.class, "clawrotate");
        airplane  = hardwareMap.get(Servo.class, "airplane");
        clawleft = hardwareMap.get(Servo.class, "clawleft");
        clawright = hardwareMap.get(Servo.class, "clawright");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        hangservo = hardwareMap.get(Servo.class, "hangservo");


        //TODO initialize the sensors and motors you added
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  armROT Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawleft.setPosition(ClosedLeft);
        clawright.setPosition(ClosedRight);
        armROT.setPosition(GroundArm);
        clawrotate.setPosition(GroundClaw);

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS); //from huskylens example
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }//makes sure the huskylens is talking to the control hub
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);// can change to other algorithms


        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();// from huskylens
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());// this gives you the data
                telemetry.addData("location?", blocks[i].x);// this gives you just x
                //TODO ensure your x values of the husky lens are appropriate to the desired areas
                //----------------------------1----------------------------\\
                if (blocks[i].x < 80) {
                    telemetry.addLine("Hooray!!! Area 1");
                    armROT.setPosition(GroundArm);
                    clawrotate.setPosition(GroundClaw);
                    clawright.setPosition(ClosedLeft);
                    clawright.setPosition(ClosedRight);
                    sleep(200);
                    move(550,550,550,550);//move away from wall
                    sleep(200);
                    turn(-515, -515, 515, 515);//turns to face zone 1
                    sleep(200);
                    move(440,440,440,440);
                    sleep(200);
                    clawleft.setPosition(OpenLeft);
                    sleep(200);
                    move(-440,-440,-440,-440);
                    sleep(1000000);
                } else {

                }
                //----------------------------2----------------------------\\
                if (blocks[i].x > 100 && blocks[i].x < 180) {
                    telemetry.addLine("Hooray!!! Area 2");
                    armROT.setPosition(GroundArm);
                    clawrotate.setPosition(GroundClaw);
                    move(1100,1100,1100,1100);//move away from wall
                    sleep(200);
                    clawleft.setPosition(OpenLeft);
                    sleep(200);
                    move(-300,-300, -300, -300);
                    sleep(100000);
                } else {

                }
                //----------------------------3----------------------------\\
                if (blocks[i].x > 180) {
                    telemetry.addLine("Hooray!!! Area 3");
                    armROT.setPosition(GroundArm);
                    clawrotate.setPosition(GroundClaw);
                    clawleft.setPosition(ClosedLeft);
                    clawright.setPosition(ClosedRight);
                    sleep(200);
                    move(300,300,300,300);//move away from wall
                    sleep(200);
                    turn(300, 300, -300, -300);//turns to face zone 3
                    sleep(200);
                    move(450,450,450,450); //450
                    sleep(200);
                    clawleft.setPosition(OpenLeft);
                    sleep(200);
                    move(-400,-400,-400,-400);
                    sleep(1000000);

                    //pixelspot = 3;
                }

            }
        }
    }


    public void arm(int LGY) {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //

        arm.setTargetPosition(LGY);
        //armY.setTargetPosition(LGY);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armY.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        arm.setPower(1);
        //armY.setPower(0.7);

        while (arm.isBusy()/* && armY.isBusy()*/) {
            sleep(25);
        }

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setPower(0);
        //armY.setPower(0);


    }

    public void move(int lf, int lb, int rf, int rb) {
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setTargetPosition(rf);
        rightBackDrive.setTargetPosition(rb);
        leftFrontDrive.setTargetPosition(lf);
        leftBackDrive.setTargetPosition(lb);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setPower(0.4);
        rightFrontDrive.setPower(0.4);
        leftFrontDrive.setPower(0.4);
        leftBackDrive.setPower(0.4);

        while (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) {
            sleep(25);

        }
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
    }
    public void turn(int lf, int lb, int rf, int rb) {
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setTargetPosition(rf);
        rightBackDrive.setTargetPosition(rb);
        leftFrontDrive.setTargetPosition(lf);
        leftBackDrive.setTargetPosition(lb);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setPower(0.4);
        rightFrontDrive.setPower(0.4);
        leftFrontDrive.setPower(0.4);
        leftBackDrive.setPower(0.4);

        while (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) {
            sleep(25);

        }
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
    }

}