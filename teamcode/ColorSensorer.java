package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous

public class ColorSensorer extends LinearOpMode {
    // Variable Declaration Code
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;

    DcMotor motorRight1;
    DcMotor motorRight2;
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    // Helper Function Code

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization Code
        colorSensor = hardwareMap.colorSensor.get("color");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color");

        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        waitForStart();
        while (opModeIsActive()) {
            // Main Loop Code
            whatColor();
            idle();
        }
        // Termination Code

    }
    public void whatColor() {
        if (colorSensor.red() > colorSensor.blue() && colorSensor.blue() < 250 && distanceSensor.getDistance(DistanceUnit.INCH) < 4) {
            telemetry.addData("I see", "red");
            telemetry.addData("The ball is" , distanceSensor.getDistance(DistanceUnit.INCH));
            //moveForward(1);
        }
        else if (colorSensor.blue() > colorSensor.red() && colorSensor.red() < 250 && distanceSensor.getDistance(DistanceUnit.INCH) < 4) {
            telemetry.addData("I see", "blue");
            telemetry.addData("The ball is" , distanceSensor.getDistance(DistanceUnit.INCH));
            //moveForward(-1);
        }
        else {
            telemetry.addData("I see", "nothing");

            //moveForward(0);
        }
        telemetry.addData("Red value is ", colorSensor.red());
        telemetry.addData("Blue value is ", colorSensor.blue());
        telemetry.update();
    }
    public void moveForward(double power) {
        motorRight1.setPower(-power);
        motorRight2.setPower(-power);
        motorLeft1.setPower(power);
        motorLeft2.setPower(power);
    }
}