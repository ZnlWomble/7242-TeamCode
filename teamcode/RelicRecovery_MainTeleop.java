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
    //DcMotor glyphPusherArm;
    //DcMotor relicArm;

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

        //glyphPusherArm = hardwareMap.dcMotor.get("glyphPusherArm");

        //relicArm = hardwareMap.dcMotor.get("relicArm");
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
        if (gamepad1.left_bumper) {
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
        else if (!gamepad1.left_bumper){
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
        else if (gamepad1.right_bumper) {
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

        //Moves the relic arm up and down
        /*if (gamepad2.dpad_up) {
            relicArm.setPower(1);
        } else if (gamepad2.dpad_down) {
            relicArm.setPower(-1);
        } else {
            relicArm.setPower(0);
        }*/
        //Runs the Glyph Tracks
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
        //Runs the glyph pushing arm
        /*if (gamepad1.a) {
            glyphPusherArm.setPower(1);
        }
        else if (gamepad1.b) {
            glyphPusherArm.setPower(-1);
        }
        else {
            glyphPusherArm.setPower(0);
        }*/
    }
}
