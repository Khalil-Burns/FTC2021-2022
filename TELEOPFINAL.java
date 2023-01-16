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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="FULL TELEOP", group="Linear Opmode")
//@Disabled
public class TELEOPFINAL extends LinearOpMode {

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
        leftIntake  = hardwareMap.get(DcMotor.class, "leftintake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightintake");
        sensorColor = hardwareMap.get(ColorSensor.class, "csensor");
        leftFront=hardwareMap.get(DcMotor.class, "leftfront");
        rightFront=hardwareMap.get(DcMotor.class, "rightfront");
        leftBack=hardwareMap.get(DcMotor.class, "leftback");
        rightBack=hardwareMap.get(DcMotor.class, "rightback");
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
//        Forward(1.5, 1);
//        RobotMoveEncoderPositions(16, 1, true, false);
//        ///////////////////
//        Forward(2, 1);
//        blocker.setPosition(0);
//        sleep(300);
//        outPusher.setPosition(0);
//        sleep(500);
//        outPusher.setPosition(0.8);
//        blocker.setPosition(0.8);
//        sleep(1000);
//        ///////////////////////
//        RobotMoveEncoderPositions(14, 1, false, true);
//        Forward(16, 1);
//        String g = "";
//        int right = 1;
//        while(true) {
//            int[] ans = colors();
//            sleep(1000);
//            if (ans[2] == 1) {
//                Forward(14, 1);
//                g = "red";
//                break;
//            } else if (ans[1] == 1) {
//                Forward(9, -1);
//                RobotMoveEncoderPositions(30, 1, false, true);
//                g = "green";
//                break;
//            } else if (ans[0] == 1) {
//                Forward(9, 1);
//                RobotMoveEncoderPositions(30, 1, true, false);
//                g = "blue";
//                break;
//            }
//            else {
//                if (right == 0) {
//                    RobotMoveEncoderPositions(1, 1, false, true);
//                    right = 2;
//                }
//                else if (right == 1) {
//                    RobotMoveEncoderPositions(4, 1, false, true);
//                    right = 2;
//                }
//                else if (right == 2) {
//                    RobotMoveEncoderPositions(1, 1, false, true);
//                    right = 3;
//                }
//                else if (right == 3) {
//                    RobotMoveEncoderPositions(1, 1, false, true);
//                    right = -1;
//                }
//                else if (right == -1) {
//                    RobotMoveEncoderPositions(4, 1, true, false);
//                    right = -2;
//                }
//                else if (right == -2) {
//                    RobotMoveEncoderPositions(1, 1, true, false);
//                    right = -3;
//                }
//                else if (right == -3) {
//                    RobotMoveEncoderPositions(1, 1, true, false);
//                    right = 1;
//                }
//            }
//        }
        ground1.setPosition(0.5);
        ground2.setPosition(0.15);
        sleep(1000);
        top1.setPosition(0.9);
        top2.setPosition(0);
        sleep(1000);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            //double leftPower;
            //double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;
            //leftPower    = Range.clip(drive + turn, -0.5, 0.5) ;
            //rightPower   = Range.clip(drive - turn, -0.5, 0.5) ;



            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            final double outPower = -1;


            if (gamepad1.a) {

                ///////////////////
                //Forward(2.5,0.2);



                blocker.setPosition(0);
                sleep(300);
                outPusher.setPosition(0);
                sleep(500);
                outPusher.setPosition(0.8);
                blocker.setPosition(0.8);


                //Forward(-2.5,0.2);
                /*leftFront.setPower(0.2);
                rightFront.setPower(0.2);
                leftBack.setPower(0.2);
                rightBack.setPower(0.2);
                sleep(1050);*/
            }
            /*else if (gamepad1.y) {
                blocker.setPosition(0);
                emergencyTapper.setPosition(0);
                sleep(800);
                outPusher.setPosition(0);
                sleep(500);
                outPusher.setPosition(0.8);
            }*/
            /*else {
                blocker.setPosition(0.65);
                //emergencyTapper.setPosition(0.95);
            }*/

            if (gamepad1.y) {
                if (emergency) {
                    emergency = false;
                }
                else {
                    emergency = true;
                }
                sleep(200);
                check();
            }
            if (emergency) {
                emergencyTapper.setPosition(0);

            }
            else {
                emergencyTapper.setPosition(1);

            }

            if(gamepad1.dpad_up){
                leftFront.setPower(0.2);
                rightFront.setPower(0.2);
                leftBack.setPower(0.2);
                rightBack.setPower(0.2);
                sleep(100);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }
            if(gamepad1.dpad_down){
                leftFront.setPower(-0.2);
                rightFront.setPower(-0.2);
                leftBack.setPower(-0.2);
                rightBack.setPower(-0.2);
                sleep(100);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }
            if(gamepad1.dpad_left){
                leftFront.setPower(-0.2);
                rightFront.setPower(0.2);
                leftBack.setPower(0.2);
                rightBack.setPower(-0.2);
                sleep(100);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }
            if(gamepad1.dpad_right){
                leftFront.setPower(0.2);
                rightFront.setPower(-0.2);
                leftBack.setPower(-0.2);
                rightBack.setPower(0.2);
                sleep(100);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }
            if(gamepad1.left_bumper){
                leftFront.setPower(-0.2);
                rightFront.setPower(0.2);
                leftBack.setPower(-0.2);
                rightBack.setPower(0.2);
                sleep(100);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }
            if(gamepad1.right_bumper){
                leftFront.setPower(0.2);
                rightFront.setPower(-0.2);
                leftBack.setPower(0.2);
                rightBack.setPower(-0.2);
                sleep(100);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }

            if(gamepad1.x) {
                // Forward(-6, -0.6);
                leftFront.setPower(0.6);
                rightFront.setPower(0.6);
                leftBack.setPower(0.6);
                rightBack.setPower(0.6);
                sleep(200);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                for (int var1 = 0; var1 < 3; var1++) {
//                    leftDrive.setPower((var1 % 1 == 0 ? outPower:0));
//                    rightDrive.setPower((var1 % 1 == 0 ? outPower:0));
                    double posG =  (var1 % 2 == 0 ? 0.5: 0);
                    double posG1 =  (var1 % 2 == 0 ? 0.15: 0.65);
                    ground1.setPosition(posG);
                    sleep(3);
                    ground2.setPosition(posG1);
                    sleep(500);
                    double posTL =  (var1 % 2 == 0 ? 0: 0.6);
                    double posTR =  (var1 % 2 == 0 ? 0.9: 0.2);
                    if (var1==1){
                      for (int i=0;i<2;i++){
                           posTL =  (i== 0 ? 0.4: 0.6);
                            posTR =  (i== 0 ? 0.45: 0.2);
                            top1.setPosition(posTR);
                            sleep(3);
                            top2.setPosition(posTL);
                            sleep(1000);
                            if(i == 1){
                                leftIntake.setPower(outPower);
                                rightIntake.setPower(outPower*0.5);
                                sleep(1000);
                            }
                        }
                    }
                    top1.setPosition(posTR);
                    sleep(3);
                    top2.setPosition(posTL);
                }
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                //////////////

                leftFront.setPower(-0.6);
                rightFront.setPower(-0.6);
                leftBack.setPower(-0.6);
                rightBack.setPower(-0.6);
                sleep(200);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }
            check();

            // Send calculated power to wheels
            //leftIntake.setPower(leftPower);
            //rightIntake.setPower(rightPower);

            /*fwdback = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
             = (fwdback + strafe - turn);
            rightfront = (fwdback - strafe + turn);
            leftback = (fwdback - strafe - turn);
            rightback = (fwdback + strafe + turn);
            max = Math.abs();
            if(Math.abs(rightfront) > max){ max = Math.abs(rightfront);}
            else if(Math.abs(rightback) > max) {max = Math.abs(rightback);}
            else if(Math.abs(leftback) > max) {max = Math.abs(leftback);}
            if(max > 1){
                leftback /= max;
                rightback /= max;
                rightfront /= max;
                 /= max;
            }
            leftFront.setPower();
            rightFront.setPower(rightfront);
            leftBack.setPower(leftback);
            rightBack.setPower(rightback);*/

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            //telemetry.update();
        }
    }
    void check(){
        fwdback = gamepad1.left_stick_y*gamepad1.left_stick_y*gamepad1.left_stick_y*gamepad1.left_stick_y*gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x*gamepad1.left_stick_x*gamepad1.left_stick_x*gamepad1.left_stick_x*gamepad1.left_stick_x;
        turn = -gamepad1.right_stick_x*gamepad1.right_stick_x*gamepad1.right_stick_x*gamepad1.right_stick_x*gamepad1.right_stick_x;
        leftfront = (-fwdback + strafe - turn);
        rightfront = (-fwdback - strafe + turn);
        leftback = (-fwdback - strafe - turn);
        rightback = (-fwdback + strafe + turn);
        max = Math.abs(leftfront);
        if(Math.abs(rightfront) > max){ max = Math.abs(rightfront);}
        else if(Math.abs(rightback) > max) {max = Math.abs(rightback);}
        else if(Math.abs(leftback) > max) {max = Math.abs(leftback);}
        if(max > 1){
            leftback /= max;
            rightback /= max;
            rightfront /= max;
            leftfront /= max;
        }
        leftFront.setPower(leftfront*0.8);
        rightFront.setPower(rightfront*0.8);
        leftBack.setPower(leftback*0.8);
        rightBack.setPower(rightback*0.8);

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
