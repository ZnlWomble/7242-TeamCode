package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="RR_MainTeleop")
//A teleop for A.R.T.H.U.R.R
public class RelicRecovery_MainTeleop extends OpMode {

    DcMotor motorRight1;
    DcMotor motorRight2;
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor glyphTrackRight;
    DcMotor glyphTrackLeft;
    DcMotor glyphPusherArm;
    boolean SNAILMODE;
    boolean SPLITCONTROL;
    DcMotor relicArm;
    Servo relicClaw;
    Servo relicBase;
    double base = 0;
    float claw = 0;
    
    Servo ballArm;
    @Override
    public void init() {
        //Drive Motors
        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        //Glyph Track Motors
        glyphTrackRight = hardwareMap.dcMotor.get("glyphTrackRight");
        glyphTrackLeft = hardwareMap.dcMotor.get("glyphTrackLeft");

        glyphPusherArm = hardwareMap.dcMotor.get("glyphPusherArm");

        relicArm = hardwareMap.dcMotor.get("relicArm");
        relicClaw = hardwareMap.servo.get("relicClaw");
        relicBase = hardwareMap.servo.get("relicBase");
        
        ballArm = hardwareMap.servo.get("armServo");
        
        //      Brake - when the motor is set to 0 it won't roll freely
        /*motorLeft1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
        
        ballArm.setPosition(0.60);
        
    }

    @Override
    public void loop() {
        ballArm.setPosition(0.60);
        double left;
        double right;
        double forward = gamepad1.left_stick_y;
        double turn = -(gamepad1.right_stick_x);
        right = forward;
        left = forward;
        //Fast Mode ;-,"
        if (gamepad1.y) {
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
        else {
        //Snail Mode _@;
            if (!SNAILMODE){
                right = right*0.75;
                left = left*0.75;
                turn = turn*0.75;
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
        //Double Snail Mode _(@);
            else if (SNAILMODE) {
                right = right/4;
                left = left/4;
                turn = turn/4;
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
        }
        //Extends the relic arm out and in
        if (gamepad2.dpad_right) {
            relicArm.setPower(1);
        } else if (gamepad2.dpad_left) {
            relicArm.setPower(-1);
        } else {
            relicArm.setPower(0);
        }
        // Opens relic claw
        if (gamepad2.x) {
            relicClaw.setPosition(0.4);//open
        }
        else if (gamepad2.y) {
            relicClaw.setPosition(1);//closed
        }
        if (gamepad2.dpad_down) {
            relicBase.setPosition(base);
            base = base + 0.01;
        }
        else if (gamepad2.dpad_up) {
            relicBase.setPosition(base);
            base = base - 0.01;
        }
        /*while (gamepad2.right_trigger > 0){
            relicBase.setPosition(gamepad2.right_trigger);
        }*/
        //Runs the Glyph Tracks
        if (SPLITCONTROL == false) {
            if (gamepad1.left_trigger > 0){
                glyphTrackLeft.setPower(0.75);
                glyphTrackRight.setPower(-0.75);
            }
            else if (gamepad1.right_trigger > 0){
                glyphTrackLeft.setPower(-0.75);
                glyphTrackRight.setPower(0.75);
            }
            else {
                glyphTrackLeft.setPower(0);
                glyphTrackRight.setPower(0);
            }
        }
        else {
                if (gamepad1.left_bumper || gamepad1.left_trigger > 0) {
                    if (gamepad1.left_bumper) {
                        glyphTrackLeft.setPower(1);
                    }
                    else if (gamepad1.left_trigger > 0) {
                        glyphTrackLeft.setPower(-1);
                    }
                    else {
                        glyphTrackLeft.setPower(0);
                    }
                }
                if (gamepad1.right_bumper || gamepad1.right_trigger > 0) {
                    if (gamepad1.right_bumper) {
                        glyphTrackRight.setPower(-1);
                    }
                    else if (gamepad1.right_trigger > 0) {
                        glyphTrackRight.setPower(1);
                    }
                    else {
                        glyphTrackRight.setPower(0);
                    }
                }
        }
        //Runs the glyph pushing arm
        if (gamepad1.b) {
            glyphPusherArm.setPower(1);
        }
        else if (gamepad1.a) {
            glyphPusherArm.setPower(-1);
        }
        else {
            glyphPusherArm.setPower(0);
        }
        if (gamepad1.right_stick_button) {
            if (SNAILMODE) {
                SNAILMODE = false;
            }
            else if (!SNAILMODE)
                SNAILMODE = true;
        }
        if (gamepad1.left_stick_button) {
            if (SPLITCONTROL) {
                SPLITCONTROL = false;
            }
            else if (!SPLITCONTROL)
                SPLITCONTROL = true;
        }
        telemetry.addData("SNAILMODE is",SNAILMODE);
        telemetry.addData("SPLITCONTROL is",SPLITCONTROL);
        telemetry.update();
    }
}
