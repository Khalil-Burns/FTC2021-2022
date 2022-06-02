package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="full", group ="Linear OpMode")
//@Disabled
public class full extends LinearOpMode {
    double curHeading = 0;
    double distance = 0;
    double TurnHeading = 0;
    double Tolerance = 2;
    double headingChange;
    double newHeading = 90;
    BNO055IMU imu;
    double absDistance = 0;
    double speed = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftb = null;
    private DcMotor rightb = null;
    private DcMotor carousel = null;
    private DistanceSensor disSensor;
    public Servo box2;
    public Servo box;
    Orientation angles;
    public int barcodePos = 0;
    public DcMotor leftIntake = null;
    public DcMotor rightIntake = null;
    double fullSpeedIntake = 0;
    double fwdback = 0;
    double strafe = 0;
    double turn = 0;
    double leftfront = 0, rightfront = 0, leftback = 0, rightback = 0;
    double max = 1;

    @Override
    public void runOpMode() {

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        box2 = hardwareMap.get(Servo.class, "box2");
        box = hardwareMap.get(Servo.class, "box");
        leftDrive = hardwareMap.get(DcMotor.class, "leftfront");
        rightDrive = hardwareMap.get(DcMotor.class, "rightfront");
        leftb = hardwareMap.get(DcMotor.class, "leftback");
        rightb = hardwareMap.get(DcMotor.class, "rightback");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        disSensor = hardwareMap.get(DistanceSensor.class, "disSensor");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Set up our telemetry dashboar
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        headingChange = angles.firstAngle;

        waitForStart();
        runtime.reset();
        //Turn(5, 1);
        //Rotate(0);


        Forward(16, 1);
        sleep(500);

        checkBarcode();
        RobotMoveEncoderPositions(17, 1, false, true);
        sleep(500);
        Forward(-12, 1);
        sleep(500);
        carousel.setPower(0.3);
        sleep(2000);
        carousel.setPower(0);
        ////////////////
        Forward(5, 1);
        sleep(500);
        RobotMoveEncoderPositions(2, 1, true, false);
        sleep(500);
        Rotate(0);
        sleep(500);
        RobotMoveEncoderPositions(42, 1, true, false);
        sleep(500);
        Rotate(180);
        sleep(1000);
        Rotate(180);

        //Turn(12, 1);
        sleep(1000);
        Forward(-17, 1); //13.5
        sleep(500);


        //barcodePos = 0;
        if (barcodePos == 0) {
            Forward(3.5, 1); //6.5
            sleep(500);
        }


        //outtake//
        box.setPosition(1);
        box2.setPosition(0.2);
        sleep(500);
        //reset//
        box2.setPosition(0.9);
        box.setPosition(0);

        sleep(500);
        if(barcodePos != 0){
            Forward(10, 1);
        }else{
            Forward(8, 1);
        }

        sleep(500);
        Turn(-7.5, -1);
        //Rotate(90);
        sleep(500);
        Forward(-55, 1);
        //sleep(2000);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightb.setDirection(DcMotor.Direction.REVERSE);
        while (opModeIsActive()) {

            leftIntake.setDirection(DcMotor.Direction.REVERSE);
            rightIntake.setDirection(DcMotor.Direction.FORWARD);

            //right trigger: gamepad1.right_trigger; returns a double between 0.0 and 1.0, depending on how jard you press the trigger

            //left and right intake to be set to full speed or 0

            leftIntake.setPower(fullSpeedIntake);
            rightIntake.setPower(fullSpeedIntake);






            if (gamepad1.dpad_right == true){
                box.setPosition(1);
            }
            if (gamepad1.dpad_left == true){
                box.setPosition(0);
            }
            if (gamepad1.right_trigger >= 0.2) {
                fullSpeedIntake = 1;
            }
            else {
                fullSpeedIntake = 0;
            }




            if(gamepad1.b){
                box.setPosition(0);
                box2.setPosition(0.9);
            }
            if (gamepad1.a){
                box.setPosition(0.3);
                box2.setPosition(0.3);

            }

            if (gamepad1.left_trigger >= 0.2){
                carousel.setPower(0.35);
            }
            else{
                carousel.setPower(0);
            }


            fwdback = gamepad2.left_stick_y;

            strafe = gamepad2.left_stick_x;
            turn = gamepad2.right_stick_x;
            leftfront = (fwdback + strafe - turn);
            rightfront = (fwdback - strafe + turn);
            leftback = (fwdback - strafe - turn);
            rightback = (fwdback + strafe + turn);
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
            leftDrive.setPower(leftfront);
            rightDrive.setPower(rightfront);
            leftb.setPower(leftback);
            rightb.setPower(rightback);
        }
    }


    public void Forward(double inches, double straightpower) {
        final double EncoderPositionsPerInch = 116.363636364;
        int encoderPositionNeeded = -1 * (int) (EncoderPositionsPerInch * inches);

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightb.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setTargetPosition(encoderPositionNeeded);
        rightDrive.setTargetPosition(encoderPositionNeeded);
        leftb.setTargetPosition(encoderPositionNeeded);
        rightb.setTargetPosition(encoderPositionNeeded);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(-straightpower);
        rightDrive.setPower(-straightpower);
        leftb.setPower(-straightpower);
        rightb.setPower(-straightpower);

        while (leftDrive.isBusy() && rightDrive.isBusy() && rightb.isBusy() && leftb.isBusy()) {

        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftb.setPower(0);
        rightb.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void RobotMoveEncoderPositions(int inches, double straightpower, boolean left, boolean right) {
        final double EncoderPositionsPerInch = 116.363636364;
        int encoderPositionNeeded = (int) (EncoderPositionsPerInch * inches);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (left == true) {
            leftDrive.setTargetPosition(encoderPositionNeeded);
            rightDrive.setTargetPosition(-encoderPositionNeeded);
            rightb.setTargetPosition(encoderPositionNeeded);
            leftb.setTargetPosition(-encoderPositionNeeded);

        }
        if (right == true) {
            leftDrive.setTargetPosition(-encoderPositionNeeded);
            rightDrive.setTargetPosition(encoderPositionNeeded);
            rightb.setTargetPosition(-encoderPositionNeeded);
            leftb.setTargetPosition(encoderPositionNeeded);

        }

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftb.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftDrive.setPower(straightpower);
        rightDrive.setPower(straightpower);
        leftb.setPower(straightpower);
        rightb.setPower(straightpower);
        while (leftDrive.isBusy() && rightDrive.isBusy() && leftb.isBusy() && rightb.isBusy()) {

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftb.setPower(0);
        rightb.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void Turn(double inches, double straightpower) {
        final double EncoderPositionsPerInch = 116.363636364;
        int encoderPositionNeeded = (int) (EncoderPositionsPerInch * inches);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(encoderPositionNeeded);

        rightDrive.setTargetPosition(-encoderPositionNeeded);
        rightb.setTargetPosition(-encoderPositionNeeded);
        leftb.setTargetPosition(encoderPositionNeeded);


        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftb.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftDrive.setPower(straightpower);
        rightDrive.setPower(straightpower);
        leftb.setPower(straightpower);
        rightb.setPower(straightpower);
        while (leftDrive.isBusy() && rightDrive.isBusy() && leftb.isBusy() && rightb.isBusy()) {

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftb.setPower(0);
        rightb.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void checkBarcode() {
        if (disSensor.getDistance(DistanceUnit.CM) <= 5) {
            barcodePos = 0;
        } else {
            barcodePos = 1;
        }
    }


    // State used for updating telemetry
    Acceleration gravity;

    private void checkOrientation() {
        // read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        curHeading = angles.firstAngle;
        telemetry.addLine(Double.toString(curHeading));
    }

    public double CalculateSpeed(double desiredHeading) {
        checkOrientation();


        absDistance = curHeading - desiredHeading;

        //speed = (-curHeading / 90);
        //distance = curHeading;
        distance = desiredHeading-Math.abs(curHeading);
        if(curHeading < 0){
            speed = -distance/60;
        }else{
            speed = distance/60;
        }
        if (speed > 1) {
            speed = 1;
        } else if (speed < -1) {
            speed = -1;
        }

        return speed;
    }


    // State used for updating telemetry
    public void Rotate(double degrees) {
        checkOrientation();
        CalculateSpeed(degrees);
        telemetry.update();
        newHeading = degrees;
        while (Math.abs(distance) >= Tolerance) {
            telemetry.update();
            CalculateSpeed(degrees);
            checkOrientation();
            leftDrive.setPower(CalculateSpeed(degrees));
            leftb.setPower(CalculateSpeed(degrees));
            rightDrive.setPower(-CalculateSpeed(degrees));
            rightb.setPower(-CalculateSpeed(degrees));
            /*if (distance >= 0) {
                leftDrive.setPower(CalculateSpeed(degrees));
                leftb.setPower(CalculateSpeed(degrees));
                rightDrive.setPower(-CalculateSpeed(degrees));
                rightb.setPower(-CalculateSpeed(degrees));

            if (distance < 0) {
                leftDrive.setPower(-CalculateSpeed(degrees));
                leftb.setPower(-CalculateSpeed(degrees));
                rightDrive.setPower(CalculateSpeed(degrees));
                rightb.setPower(CalculateSpeed(degrees));
            }*/
            checkOrientation();
            telemetry.update();
        }

        leftDrive.setPower(0);
        leftb.setPower(0);
        rightDrive.setPower(0);
        rightb.setPower(0);

        /*if (Math.abs(distance) >= Tolerance) {
            Rotate(degrees);
        }*/
    }
}
