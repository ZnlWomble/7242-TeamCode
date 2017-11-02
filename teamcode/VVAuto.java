package org.firstinspires.ftc.teamcode;
//Auto for URN
// Import Statements
 import com.qualcomm.hardware.bosch.BNO055IMU;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.ColorSensor;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.GyroSensor;

 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
 import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous

public class VVAuto extends LinearOpMode {
    // Variable Declaration Code
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor leftShootMotor;
    DcMotor rightShootMotor;
    DcMotor whiskMotor;
    BNO055IMU imu;
    int TOLERANCE = 5;
    boolean goalReached;
    Orientation angles;
    // Motor Align Variables
    private static final int MAX_MOTOR_RPM = 77;
    boolean speedCheckTrigger = false;
    double speedCheckStartTime;
    int currentCounts;
    // Helper Function Code

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialization Code
        motorFrontRight = hardwareMap.dcMotor.get("frontRightMotor");
        motorBackRight = hardwareMap.dcMotor.get("backRightMotor");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        motorBackLeft = hardwareMap.dcMotor.get("backLeftMotor");
        leftShootMotor = hardwareMap.dcMotor.get("leftShootMotor");
        rightShootMotor = hardwareMap.dcMotor.get("rightShootMotor");
        whiskMotor = hardwareMap.dcMotor.get("whiskMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //colorSensor = hardwareMap.colorSensor.get("color");
        waitForStart();
        while (opModeIsActive()) {

            // Main Loop Code
            forwardWithTime(500, 0.5);
            shoot();
            forwardWithTime(2100, 0.5);
            stop();
            /*while (colorSensor.alpha() < 20) {
                moveForward(1000, 0.5);
            }
            while (opModeIsActive()) {

            }*/
            idle();
        }
        // Termination Code
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
    }
    public void turnLeft(double turn, double power) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if ((angles.firstAngle <= -turn + TOLERANCE) && (angles.firstAngle >= -turn - TOLERANCE)) {
                goalReached = true;
            }
            if (goalReached) {
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
            }
            if (!goalReached) {
                motorFrontRight.setPower(power);
                motorBackRight.setPower(power);
                motorFrontLeft.setPower(-power);
                motorBackLeft.setPower(power);
            }
        }
    public void turnRight(double turn, double power) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if ((angles.firstAngle <= turn + TOLERANCE) && (angles.firstAngle >= turn - TOLERANCE)) {
                goalReached = true;
            }
            if (goalReached) {
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
            }
            if (!goalReached) {
                motorFrontRight.setPower(power);
                motorBackRight.setPower(power);
                motorFrontLeft.setPower(-power);
                motorBackLeft.setPower(power);
            }
        }
    public void moveForward(int position, double power) {
        motorFrontRight.setTargetPosition(-position);
        motorBackRight.setTargetPosition(-position);
        motorFrontLeft.setTargetPosition(position);
        motorBackLeft.setTargetPosition(position);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(power);
        while (motorFrontLeft.getCurrentPosition() < motorFrontLeft.getTargetPosition() || motorFrontRight.getCurrentPosition() < motorFrontRight.getTargetPosition()
                || motorBackLeft.getCurrentPosition()  < motorBackLeft.getTargetPosition() || motorBackRight.getCurrentPosition() < motorBackRight.getCurrentPosition()) {
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void shoot() {
        alignMotorSpeed(rightShootMotor, leftShootMotor);
        leftShootMotor.setPower(-0.6);
        rightShootMotor.setPower(-0.6);
        sleep(1100);
        runWhisk();
        runWhisk();
        leftShootMotor.setPower(0);
        rightShootMotor.setPower(0);
        whiskMotor.setPower(0);
    }
    public void forwardWithTime(long time, double power) {
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(power);
        sleep(time);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
    }
    public void runWhisk(){
        whiskMotor.setPower(1);
        sleep(500);
        whiskMotor.setPower(0);
    }
    public int motorSpeed(DcMotor motor)
    {
        //Gets the Speed (NOT POWER) of the motor given as a parameter
        //We use this to see the true speed of the motors, not just what we tell the power to be
        if(speedCheckTrigger == false)
        {
            speedCheckStartTime = time;
            currentCounts = motor.getCurrentPosition();
            speedCheckTrigger = true; 
        }
        if(time <= speedCheckStartTime + 1)
        {
            currentCounts = motor.getCurrentPosition() - currentCounts;
        }
        else
        {
            return currentCounts;
        }
        return 0;
    }
    public void alignMotorSpeed(DcMotor motorOne, DcMotor motorTwo)
    {
        //Takes the absolute value of the motors motorOne and motorTwo and finds the mid point between them
        if(Math.abs(motorSpeed(motorOne)) > Math.abs(motorSpeed(motorTwo)))
        {
            // if motorOne is spinning faster, we slow it down to meet motorTwo
            // motorTwo speeds up to meet motorOne at the midpoint between their speeds
            telemetry.addData("Aligning Motor Speed", "MotorOne > MotorTwo");
            motorOne.setPower(motorOne.getPower() - ((rpmToPowerConverter(motorSpeed(motorOne)) - rpmToPowerConverter(motorSpeed(motorTwo))) / 2));
            motorTwo.setPower(motorTwo.getPower() + ((rpmToPowerConverter(motorSpeed(motorOne)) - rpmToPowerConverter(motorSpeed(motorTwo))) / 2));
        }
        else if (Math.abs(motorSpeed(motorOne)) < Math.abs(motorSpeed(motorTwo)))
        {
            // if motorTwo is spinning faster, we slow it down to meet motorOne
            // motorOne speeds up to meet motorTwo at the midpoint between their speeds
            telemetry.addData("Aligning Motor Speed", "MotorTwo > MotorOne");
            motorTwo.setPower(motorTwo.getPower() - ((rpmToPowerConverter(motorSpeed(motorTwo)) - rpmToPowerConverter(motorSpeed(motorOne))) / 2));
            motorOne.setPower(motorOne.getPower() + ((rpmToPowerConverter(motorSpeed(motorTwo)) + rpmToPowerConverter(motorSpeed(motorOne))) / 2));
        }
        else
        {
            // telemetry was added to help the drivers see when the motors are aligned
            // telemetry was also a great help during testing
            telemetry.addData("Motor Speeds are Aligned", "MotorOne === MotorTwo");
        }
    }
    public double rpmToPowerConverter(int numToConvert)
    {
        //Converts the speed number we are getting to a power that we can set the motor's target power to
        //Uses a constant which we found by setting the power of one of our motors to 1 and calling motorSpeed() on it
        double convertedNum = (double)(numToConvert / MAX_MOTOR_RPM);
        return convertedNum;
    }
}