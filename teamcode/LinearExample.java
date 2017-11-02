package org.firstinspires.ftc.teamcode;
// Import Statements

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
@Disabled

public class LinearExample extends LinearOpMode {
    // Variable Declaration Code

    // Helper Function Code

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization Code

        waitForStart();
        while (opModeIsActive()) {
            // Main Loop Code

            idle();
        }
        // Termination Code

    }

}