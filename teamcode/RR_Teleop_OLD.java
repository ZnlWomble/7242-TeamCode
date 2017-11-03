package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
//A teleop for A.R.T.H.U.R.R
public class RR_Teleop_OLD extends OpMode {

    DcMotor motorRight1;
    DcMotor motorRight2;
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor liftMotor;
    Servo leftArmServo;
    Servo rightArmServo;

    @Override
    public void init() {
        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        leftArmServo = hardwareMap.servo.get("leftArmServo");
        rightArmServo = hardwareMap.servo.get("rightArmServo");
    }

    @Override
    public void loop() {
        double left;
        double right;
        double forward = gamepad1.left_stick_y;
        double turn = -(gamepad1.right_stick_x);
        right = forward;
        left = forward;
        left = left * 1;
        //Fast Mode ;-,"
        if (!gamepad1.left_bumper) {
            if (turn > 0.1 || turn < -0.1){
                    motorRight1.setPower(turn);
                    motorRight2.setPower(turn);
                    motorLeft1.setPower(turn);
                    motorLeft2.setPower(turn);
            }
            else {
                motorRight1.setPower(-right);
                motorRight2.setPower(-right);
                motorLeft1.setPower(left);
                motorLeft2.setPower(left);
            } 
        }
        //Snail Mode _@;
        else {
            right = right/2;
            left = left/2;
            turn = turn/2;
            if (turn > 0.1 || turn < -0.1){
                motorRight1.setPower(turn);
                motorRight2.setPower(turn);
                motorLeft1.setPower(turn);
                motorLeft2.setPower(turn);
            }
            else {
                motorRight1.setPower(-right);
                motorRight2.setPower(-right);
                motorLeft1.setPower(left);
                motorLeft2.setPower(left);
            }
        }
        //Controls
        if (gamepad1.y) {
            openArms();
        }
        if (gamepad1.b) {
            closeArms();
        }
        if (gamepad1.a) {
            openArmsALittle();
        }
        if (gamepad1.dpad_up) {
            liftMotor.setPower(1);
        }
        else if (gamepad1.dpad_down) {
            liftMotor.setPower(-1);
        }
        else {
            liftMotor.setPower(0);
        }
    }
    //Glyph Grabber Functions
    public void openArms() {
        rightArmServo.setPosition(0.97);
        leftArmServo.setPosition(0.13);
    }
    public void closeArms(){
        rightArmServo.setPosition(0.56);
        leftArmServo.setPosition(0.45);
    }
    public void openArmsALittle() {
        rightArmServo.setPosition(0.71);
        leftArmServo.setPosition(0.30);
    }
}