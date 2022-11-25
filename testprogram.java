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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="CURRENT PROGRAM ", group="Linear Opmode")
//@Disabled
public class testprogram extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;
    private Servo ground1 = null, ground2 = null, top1=null, top2=null, outPusher = null, blocker = null;
    int /*var1 =0, */var2 = 0;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "leftintake");
        rightDrive = hardwareMap.get(DcMotor.class, "rightintake");
        ground1 = hardwareMap.get(Servo.class, "groundleft");
        ground2 = hardwareMap.get(Servo.class, "groundright");
        top1 = hardwareMap.get(Servo.class, "topleft");
        top2 = hardwareMap.get(Servo.class, "topright");
        outPusher = hardwareMap.get(Servo.class, "outPusher");
        blocker = hardwareMap.get(Servo.class, "blocker");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -0.5, 0.5) ;
            rightPower   = Range.clip(drive - turn, -0.5, 0.5) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            final double outPower = -0.50;
            if (gamepad1.a) {
                blocker.setPosition(0);
                sleep(300);
                outPusher.setPosition(0);
                sleep(500);
                outPusher.setPosition(0.7);
            }
            else {
                blocker.setPosition(0.65);
            }
            if(gamepad1.x) {
//                leftDrive.setPower(outPower);
//                rightDrive.setPower(outPower);
//                //sleep(2000);
//                double posG =  (var1 == 0 ? 0.5: 0);
//                double posG1 =  (var1 == 0 ? 0.15: 0.65);
//                ground1.setPosition(posG);
//                ground2.setPosition(posG1);
//
//                sleep(250);
//                double posTR =  (var1 == 0 ? 1: 0.2);
//                double posTL =  (var1 == 0 ? 0: 0.6);
//                top1.setPosition(posTR);
//                top2.setPosition(posTL);
//                var1 = (var1 == 0 ? 1: 0);
//
//                sleep(150);
                leftDrive.setPower(outPower);
                rightDrive.setPower(outPower);
                for (int var1 = 0; var1 < 3; var1++) {
//                    leftDrive.setPower((var1 % 1 == 0 ? outPower:0));
//                    rightDrive.setPower((var1 % 1 == 0 ? outPower:0));
                    double posG =  (var1 % 2 == 0 ? 0.5: 0);
                    double posG1 =  (var1 % 2 == 0 ? 0.15: 0.65);
                    ground1.setPosition(posG);
                    sleep(25);
                    ground2.setPosition(posG1);
                    sleep(200);
                    double posTL =  (var1 % 2 == 0 ? 0: 0.6);
                    double posTR =  (var1 % 2 == 0 ? 0.9: 0.2);
                    if (var1==1){
                        for (int i=0;i<2;i++){
                            posTL =  (i== 0 ? 0.3: 0.6);
                            posTR =  (i== 0 ? 0.55: 0.2);
                            top1.setPosition(posTR);
                            sleep(3);
                            top2.setPosition(posTL);
                            sleep(750);
                        }
                    }
                    top1.setPosition(posTR);
                    sleep(3);
                    top2.setPosition(posTL);
                    sleep(750);
                }
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
