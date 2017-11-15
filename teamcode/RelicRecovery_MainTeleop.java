package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
    }

    @Override
    public void loop() {
        double left;
        double right;
        double forward = gamepad1.left_stick_y;
        double turn = -(gamepad1.right_stick_x);
        right = forward;
        left = forward;
        //Fast Mode ;-,"
        /*if (!SNAILMODE) {
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
        }*/
        //Snail Mode _@;
        if (!SNAILMODE){
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

        //Extends the relic arm out and in
        if (gamepad2.dpad_up) {
            relicArm.setPower(0.25);
        } else if (gamepad2.dpad_down) {
            relicArm.setPower(-0.25);
        } else {
            relicArm.setPower(0);
        }
        //Runs the Glyph Tracks
        //  If SPLITCONTROL is active the tracks run independently
        if (SPLITCONTROL){
            if (!gamepad1.right_bumper || !gamepad1.left_bumper || gamepad1.right_trigger < 0.1 || gamepad1.left_trigger < 0.1) {
                if (gamepad1.left_trigger > 0) {
                    glyphTrackLeft.setPower(1);
                } else if (gamepad1.left_bumper) {
                    glyphTrackLeft.setPower(-1);
                }
                if (gamepad1.right_bumper) {
                    glyphTrackRight.setPower(-1);
                } else if (gamepad1.right_trigger > 0) {
                    glyphTrackRight.setPower(1);
                }
            }
        }
        // If SPLITCONTROL isn't active the tracks run together
        else {
            if (gamepad1.left_trigger > 0) {
                glyphTrackLeft.setPower(1);
                glyphTrackRight.setPower(-1);
            }
            else if (gamepad1.right_trigger > 0) {
                glyphTrackLeft.setPower(-1);
                glyphTrackRight.setPower(1);
            }
            else {
                glyphTrackLeft.setPower(0);
                glyphTrackRight.setPower(0);
            }
        }
        //Runs the glyph pushing arm
        if (gamepad1.a) {
            glyphPusherArm.setPower(.3);
        }
        else if (gamepad1.b) {
            glyphPusherArm.setPower(-.3);
        }
        else {
            glyphPusherArm.setPower(0);
        }
        //Toggles SNAILMODE
        if (gamepad1.right_stick_button) {
            if (SNAILMODE) {
                SNAILMODE = false;
            }
            else if (!SNAILMODE)
                SNAILMODE = true;
        }
        //Toggles SPLITCONTROL
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
