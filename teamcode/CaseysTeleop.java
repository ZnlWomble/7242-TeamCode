package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp(name="MainTeleOp: Iterative OpMode", group="Iterative Opmode")

public class CaseysTeleop extends OpMode
{
    DcMotor leftShootMotor, rightShootMotor, whiskMotor, frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, slideRailMotor;
    Servo leftButtonPushServo, rightButtonPushServo;
   // GyroSensor gyro;
    float mmPerInch = 25.4f;
    float mmBotWidth = 18 * mmPerInch;
    float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;
    double speedCheckStartTime;
    int currentCounts;
    private static final int MAX_MOTOR_RPM = 77;
    boolean speedCheckTrigger = false;
    private int counter;

    @Override
    public void init()
    {
        leftShootMotor = hardwareMap.dcMotor.get("leftShootMotor");
        rightShootMotor = hardwareMap.dcMotor.get("rightShootMotor");
        whiskMotor = hardwareMap.dcMotor.get("whiskMotor");
        slideRailMotor = hardwareMap.dcMotor.get("slideRailMotor");
        //leftButtonPushServo = hardwareMap.servo.get("leftButtonPushServo");
        //rightButtonPushServo = hardwareMap.servo.get("rightButtonPushServo");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
       // gyro = hardwareMap.gyroSensor.get("gyro");
        //gyro.calibrate();
        //rightShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        //leftShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        whiskMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //gyro.calibrate();
    }

    @Override
    public void loop()
    {
       /* if(gamepad1.a)
        {
            if(gyro.getHeading() > 5 && gyro.getHeading() < 180)
            {
                frontRightMotor.setPower(-.55);
                backRightMotor.setPower(.55);
                frontLeftMotor.setPower(.5);
                backLeftMotor.setPower(-.5);
            }
            else if (gyro.getHeading() > 180)
            {
                frontRightMotor.setPower(-.5);
                frontLeftMotor.setPower(.55);
                backRightMotor.setPower(.5);
                backLeftMotor.setPower(-.55);
            }
            else if (gyro.getHeading() <= 5)
            {
                frontRightMotor.setPower(-.5);
                frontLeftMotor.setPower(.5);
                backLeftMotor.setPower(-.5);
                backRightMotor.setPower(.5);
            }
        }

        /*
        if(gamepad1.dpad_left)
        {
            leftButtonPushServo.setPosition(.25);
        }
        if(gamepad1.dpad_left == false)
        {
            leftButtonPushServo.setPosition(0);
        }
        if(gamepad1.dpad_right)
        {
            rightButtonPushServo.setPosition(.75);
        }
        if(gamepad1.dpad_right == false)
        {
            rightButtonPushServo.setPosition(1);
        }
        */
        if(gamepad2.a==true)
        {
            leftShootMotor.setPower(-.6);
            rightShootMotor.setPower(-.6);
            //alignMotorSpeed(leftShootMotor, rightShootMotor);
        }
        if(gamepad2.b==true)
        {
            leftShootMotor.setPower(1);
            rightShootMotor.setPower(1);
        }
        if (gamepad2.x==true)
        {
            leftShootMotor.setPower(-.75);
            rightShootMotor.setPower(-.75);
            //alignMotorSpeed(leftShootMotor, rightShootMotor);
        }
        if (gamepad2.y==true)
        {
            leftShootMotor.setPower(-.3);
            rightShootMotor.setPower(-.3);
            //alignMotorSpeed(leftShootMotor, rightShootMotor);
        }
        if (gamepad2.a == false && gamepad2.x == false && gamepad2.y == false)
        {
            // Turn off Shooters
            leftShootMotor.setPower(0);
            rightShootMotor.setPower(0);
        }
        /*if(gamepad2.b == true && gamepad2.x == false && gamepad2.y == false && gamepad2.a == false)
        {
            leftShootMotor.setPower(-.5);
            rightShootMotor.setPower(-.5);
            alignMotorSpeed(leftShootMotor, rightShootMotor);
        }
        */
        if(gamepad2.dpad_up && gamepad2.dpad_down == false)
        {
            slideRailMotor.setPower(1);
        }
        else if(gamepad2.dpad_down && gamepad2.dpad_up == false)
        {
            slideRailMotor.setPower(-1);
        }
        else if(gamepad2.dpad_down == false && gamepad2.dpad_up == false)
        {
            slideRailMotor.setPower(0);
        }

        if (gamepad2.right_trigger > 0)
        {
            // Whisk Forward
            whiskMotor.setPower(1);
        }
        if (gamepad2.left_trigger > 0)
        {
            //Whisk Reverse
            whiskMotor.setPower(-1);
        }
        if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0)
        {
            // Turn off Whisk
            whiskMotor.setPower(0);
        }

        // Positive slope diagonal
        if (gamepad1.right_bumper && gamepad1.left_stick_y != 0)
        {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(-gamepad1.left_stick_y);
            backLeftMotor.setPower(-gamepad1.left_stick_y);
            //alignMotorSpeed(frontRightMotor, backLeftMotor);
            backRightMotor.setPower(0);
        }
        // Negative slope diagonal
        else if (gamepad1.left_bumper && gamepad1.left_stick_y != 0)
        {
            frontLeftMotor.setPower(-gamepad1.left_stick_y);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(-gamepad1.left_stick_y);
            //alignMotorSpeed(frontLeftMotor, backRightMotor);
            backLeftMotor.setPower(0);
        }

        // FORWARD / BACKWARD
        if (gamepad1.left_stick_y != 0 && gamepad1.left_bumper == false && gamepad1.right_bumper == false && gamepad1.b == false)
        {
            frontRightMotor.setPower(gamepad1.left_stick_y);
            backRightMotor.setPower(gamepad1.left_stick_y);
            frontLeftMotor.setPower(gamepad1.left_stick_y);
            backLeftMotor.setPower(-gamepad1.left_stick_y);
            //alignFourMotorSpeed(frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor);
        }
        //STOP
        if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0 && gamepad1.a == false)
        {
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            //alignFourMotorSpeed(frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor);
        }

        //POINT TURN RIGHT
        if (gamepad1.right_stick_x > 0 && gamepad1.left_stick_y == 0 && gamepad1.left_bumper == false && gamepad1.right_bumper == false)
        {
            frontRightMotor.setPower(gamepad1.right_stick_x);
            backRightMotor.setPower(gamepad1.right_stick_x);
            frontLeftMotor.setPower(-gamepad1.right_stick_x);
            backLeftMotor.setPower(gamepad1.right_stick_x);
            //alignFourMotorSpeed(frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor);
        }
        //POINT TURN LEFT
        else if (gamepad1.right_stick_x < 0 && gamepad1.right_stick_y == 0 && gamepad1.left_bumper == false && gamepad1.right_bumper == false)
        {
            frontRightMotor.setPower(gamepad1.right_stick_x);
            frontLeftMotor.setPower(-gamepad1.right_stick_x);
            backLeftMotor.setPower(gamepad1.right_stick_x);
            backRightMotor.setPower(gamepad1.right_stick_x);
            //alignFourMotorSpeed(frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor);
        }

        //STRAFE LEFT
        if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0)
        {
            frontRightMotor.setPower(-gamepad1.left_trigger);
            backRightMotor.setPower(gamepad1.left_trigger);
            frontLeftMotor.setPower(gamepad1.left_trigger);
            backLeftMotor.setPower(gamepad1.left_trigger);
            //alignFourMotorSpeed(frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor);
        }
        //STRAFE RIGHT
        if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0)
        {
            frontRightMotor.setPower(gamepad1.right_trigger);
            backRightMotor.setPower(-gamepad1.right_trigger);
            frontLeftMotor.setPower(-gamepad1.right_trigger);
            backLeftMotor.setPower(-gamepad1.right_trigger);
            //alignFourMotorSpeed(frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor);
        }

        //ROTATE ON BACK AXIS RIGHT
        if (gamepad1.b && gamepad1.left_stick_x > 0 && gamepad1.right_stick_y == 0 && gamepad1.right_stick_x == 0)
        {
            frontLeftMotor.setPower(gamepad1.left_stick_x);
            frontRightMotor.setPower(-gamepad1.left_stick_x);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            //alignMotorSpeed(frontLeftMotor, frontRightMotor);
        }
        //ROTATE ON BACK AXIS LEFT
        else if (gamepad1.b && gamepad1.left_stick_x < 0 && gamepad1.right_stick_y == 0 && gamepad1.right_stick_x == 0)
        {
            frontLeftMotor.setPower(-gamepad1.left_stick_x);
            frontRightMotor.setPower(gamepad1.left_stick_x);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            //alignMotorSpeed(frontLeftMotor, frontRightMotor);
        }
    }

    public void alignFourMotorSpeed(DcMotor motorOne, DcMotor motorTwo, DcMotor motorThree, DcMotor motorFour)
    {
        if(motorSpeed(motorOne) != motorSpeed(motorTwo))
        {
            alignMotorSpeed(motorOne, motorTwo);
        }
        if(motorSpeed(motorThree) != motorSpeed(motorFour))
        {
            alignMotorSpeed(motorThree, motorFour);
        }
        if(motorSpeed(motorOne) == motorSpeed(motorTwo) && motorSpeed(motorThree) == motorSpeed(motorFour) && motorSpeed(motorTwo) != motorSpeed(motorThree))
        {
            alignMotorSpeed(motorTwo, motorThree);
        }
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

    public double rpmToPowerConverter(int numToConvert)
    {
        //Converts the speed number we are getting to a power that we can set the motor's target power to
        //Uses a constant which we found by setting the power of one of our motors to 1 and calling motorSpeed() on it
        double convertedNum = (double)(numToConvert / MAX_MOTOR_RPM);
        return convertedNum;
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

}