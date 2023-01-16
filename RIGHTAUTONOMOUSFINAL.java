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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="RIGHT AUTONOMOUS", group="Autonomous")
//@Disabled
public class RIGHTAUTONOMOUSFINAL extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    ColorSensor sensorColor;
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    private DcMotor leftFront=null;
    private DcMotor rightFront=null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;
    private Servo ground1 = null, ground2 = null, top1=null, top2=null, outPusher = null, blocker = null, emergencyTapper = null;
    int /*var1 =0, */var2 = 0;
    double fullSpeedIntake = 0;
    float hsvValues[] = {0F, 0F, 0F};
    final double scale = 255;
    double fwdback = 0;
    double strafe = 0;
    double turn = 0;
    double leftfront = 0, rightfront = 0, leftback = 0, rightback = 0;
    double max = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftIntake = hardwareMap.get(DcMotor.class, "leftintake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightintake");
        sensorColor = hardwareMap.get(ColorSensor.class, "csensor");
        leftFront = hardwareMap.get(DcMotor.class, "leftfront");
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        rightBack = hardwareMap.get(DcMotor.class, "rightback");
        ground1 = hardwareMap.get(Servo.class, "groundleft");
        ground2 = hardwareMap.get(Servo.class, "groundright");
        top1 = hardwareMap.get(Servo.class, "topleft");
        top2 = hardwareMap.get(Servo.class, "topright");
        outPusher = hardwareMap.get(Servo.class, "outPusher");
        blocker = hardwareMap.get(Servo.class, "blocker");
        emergencyTapper = hardwareMap.get(Servo.class, "emergencyTapper");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //change this so it fits right
        boolean emergency = false;
        Forward(1.5, 1);
        RobotMoveEncoderPositions(16, 1, true, false);
        ///////////////////
        Forward(4, 1);
        blocker.setPosition(0);
        sleep(300);
        outPusher.setPosition(0);
        sleep(500);
        outPusher.setPosition(0.8);
        blocker.setPosition(0.8);
        sleep(1000);
        ///////////////////////
        RobotMoveEncoderPositions(10, 1, false, true);
        Forward(15, 1);
        String g = "";
        int right = 1;
        while (true) {
            int[] ans = colors();
            sleep(1000);
            if (ans[0] == 1) {
                Forward(17, 1);
                g = "red";
                break;
            } else if (ans[1] == 1) {
                Forward(11, -1);
                RobotMoveEncoderPositions(27, 1, false, true);
                g = "green";
                break;
            } else if (ans[2] == 1) {
                Forward(11, 1);
                RobotMoveEncoderPositions(27, 1, true, false);
                g = "blue";
                break;
            } else {
                if (right == 0) {
                    RobotMoveEncoderPositions(1, 1, false, true);
                    right = 2;
                } else if (right == 1) {
                    RobotMoveEncoderPositions(4, 1, false, true);
                    right = 2;
                } else if (right == 2) {
                    RobotMoveEncoderPositions(1, 1, false, true);
                    right = 3;
                } else if (right == 3) {
                    RobotMoveEncoderPositions(1, 1, false, true);
                    right = -1;
                } else if (right == -1) {
                    RobotMoveEncoderPositions(4, 1, true, false);
                    right = -2;
                } else if (right == -2) {
                    RobotMoveEncoderPositions(1, 1, true, false);
                    right = -3;
                } else if (right == -3) {
                    RobotMoveEncoderPositions(1, 1, true, false);
                    right = 1;
                }
            }
        }
        ground1.setPosition(0.5);

        ground2.setPosition(0.15);
        sleep(1000);
        top1.setPosition(0.9);
        top2.setPosition(0);
        sleep(1000);
    }
  public int[] colors(){
        Color.RGBToHSV((int) (sensorColor.red() * scale),
                (int) (sensorColor.green() * scale),
                (int) (sensorColor.blue() * scale),
                hsvValues);
        int hue = (int)hsvValues[0];
        int r =0, g = 0, b = 0;
        //RED CASE
        if((hue >= 344 && hue <= 360) || (hue >= 0 && hue <= 20)){
            r=1;
        }else if((hue >= 80 && hue <= 138)){
            g = 1;
        }else if(hue >= 180 && hue <= 270){
            b= 1;
        }
        int[] vals = {r, g, b};
        return vals;

    }
    public void Forward(double inches, double straightpower) {
        final double EncoderPositionsPerInch = 116.363636364;
        int encoderPositionNeeded = -1 * (int) (EncoderPositionsPerInch * inches);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition(encoderPositionNeeded);
        rightFront.setTargetPosition(encoderPositionNeeded);
        leftBack.setTargetPosition(encoderPositionNeeded);
        rightBack.setTargetPosition(encoderPositionNeeded);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(straightpower);
        rightFront.setPower(straightpower);
        leftBack.setPower(straightpower);
        rightBack.setPower(straightpower);

        while (leftFront.isBusy() && rightFront.isBusy() && rightBack.isBusy() && leftBack.isBusy()) {

        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void RobotMoveEncoderPositions(int inches, double straightpower, boolean left, boolean right) {
        final double EncoderPositionsPerInch = 116.363636364;
        int encoderPositionNeeded = (int) (EncoderPositionsPerInch * inches);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (left == true) {
            leftFront.setTargetPosition(encoderPositionNeeded);
            rightFront.setTargetPosition(-encoderPositionNeeded);
            rightBack.setTargetPosition(encoderPositionNeeded);
            leftBack.setTargetPosition(-encoderPositionNeeded);

        }
        if (right == true) {
            leftFront.setTargetPosition(-encoderPositionNeeded);
            rightFront.setTargetPosition(encoderPositionNeeded);
            rightBack.setTargetPosition(-encoderPositionNeeded);
            leftBack.setTargetPosition(encoderPositionNeeded);

        }

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftFront.setPower(straightpower);
        rightFront.setPower(straightpower);
        leftBack.setPower(straightpower);
        rightBack.setPower(straightpower);
        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {

        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}

