package org.firstinspires.ftc.teamcode;
// Import Statements

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous


public class EncoderTest extends LinearOpMode {
    // Variable Declaration Code
    DcMotor motorRight1;
    DcMotor motorRight2;
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    // Helper Function Code

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization Code
        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        waitForStart();
        while (opModeIsActive()) {
            // Loop Main Code
            moveForward(1000, 0.5);
            idle();
            stop();
        }
        // Termination Code

    }
    public void moveForward(int position, double power) {
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorRight1.setTargetPosition(position);
        motorRight2.setTargetPosition(position);
        motorLeft1.setTargetPosition(position);
        motorLeft2.setTargetPosition(position);

        while (motorRight1.getCurrentPosition() < motorRight1.getTargetPosition() || motorRight2.getCurrentPosition() < motorRight2.getCurrentPosition()
                || motorLeft1.getCurrentPosition() < motorLeft1.getTargetPosition() || motorLeft2.getCurrentPosition()  < motorLeft2.getTargetPosition() ) {
            motorRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorRight1.setPower(-power);
            motorRight2.setPower(-power);
            motorLeft1.setPower(power);
            motorLeft2.setPower(power);

            telemetry.addData("The position is", motorRight1.getCurrentPosition());
            telemetry.update();
        }
        motorRight1.setPower(0);
        motorRight2.setPower(0);
        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
    }
}