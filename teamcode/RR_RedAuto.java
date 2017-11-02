package org.firstinspires.ftc.teamcode;
// Import Statements

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous

public class RR_RedAuto extends LinearOpMode {
    // Variable Declaration Code
    DcMotor motorRight1;
    DcMotor motorRight2;
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor glyphTrackRight;
    DcMotor glyphTrackLeft;
    DcMotor glyphPusherArm;
    DcMotor relicArm;
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;
    //BNO055IMU imu;
    Servo ballArm;
    VuforiaLocalizer vuforia;
    boolean goalReached;
    double TOLERANCE;
    RelicRecoveryVuMark cryptobox = RelicRecoveryVuMark.UNKNOWN;

    //  Running Glyph Tracks
    //      The first time the tracks are run they use encoders
    public void initialRunGlyphTrack(double power) {
        glyphTrackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphTrackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        glyphTrackRight.setTargetPosition(500);
        glyphTrackLeft.setTargetPosition(-500);

        glyphTrackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glyphTrackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        glyphTrackRight.setPower(power);
        glyphTrackLeft.setPower(power);

        while (glyphTrackRight.isBusy() & glyphTrackLeft.isBusy()) {
            //wait
        }
        glyphTrackRight.setPower(0);
        glyphTrackLeft.setPower(0);
    }
    //      After that they use time
    public void runGlyphTrack(double power, long time) {
        glyphTrackRight.setPower(power);
        glyphTrackLeft.setPower(-power);
    }

    //  Hitting the Ball
    public void hitBall() {
        ballArm.setPosition(0);
        sleep(1250);
        if (colorSensor.blue() > colorSensor.red() && colorSensor.red() < 250 && distanceSensor.getDistance(DistanceUnit.INCH) < 4) {
            moveForward(70, -0.25);
            ballArm.setPosition(0.75);
            sleep(100);
            moveForward(-140, 0.25);
            //telemetry.addData("The ball is","Blue" );
        } else if (colorSensor.red() > colorSensor.blue() && colorSensor.blue() < 250 && distanceSensor.getDistance(DistanceUnit.INCH) < 4) {
            moveForward(-70, 0.25);
            sleep(100);
            ballArm.setPosition(0.75);
            //telemetry.addData("The ball is","Red" );
        } else {
            ballArm.setPosition(0.75);
            sleep(100);
            moveForward(-70, 0.25);
            telemetry.addData("I didn't see a", "ball");
        }
        telemetry.update();
    }

    //  Movement Functions
    public void turnRightWithEncoders(double power) {
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorRight1.setTargetPosition(-500);
        motorRight2.setTargetPosition(-500);
        motorLeft1.setTargetPosition(500);
        motorLeft2.setTargetPosition(500);

        motorRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRight1.setPower(-power);
        motorRight2.setPower(-power);
        motorLeft1.setPower(power);
        motorLeft2.setPower(power);

        while (motorLeft1.isBusy() & motorLeft2.isBusy() & motorRight1.isBusy() & motorRight2.isBusy()) {
            //wait
        }
        reset();
    }

    public void turnLeftWithEncoders(double power) {
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorRight1.setTargetPosition(500);
        motorRight2.setTargetPosition(500);
        motorLeft1.setTargetPosition(-500);
        motorLeft2.setTargetPosition(-500);

        motorRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRight1.setPower(-power);
        motorRight2.setPower(-power);
        motorLeft1.setPower(power);
        motorLeft2.setPower(power);

        while (motorLeft1.isBusy() & motorLeft2.isBusy() & motorRight1.isBusy() & motorRight2.isBusy()) {
            //wait
        }
        reset();
    }
    public void turnAroundWithEncoders(double power) {
        turnLeftWithEncoders(power);
        turnLeftWithEncoders(power);
    }

    public void goToCryptobox(double power, RelicRecoveryVuMark glyph) {
        if (glyph == RelicRecoveryVuMark.LEFT) {
            moveForward(-1098, power);
            telemetry.addData("Moving to", "left");
        } else if (glyph == RelicRecoveryVuMark.CENTER) {
            moveForward(-775, power);
            telemetry.addData("Moving to", "center");
        } else if (glyph == RelicRecoveryVuMark.RIGHT) {
            moveForward(-482, power);
            telemetry.addData("Moving to", "right");
        } else {
            moveForward(-482, power);
            telemetry.addData("There's no VuMark", "I'm just gonna go to the RIGHT");
        }
        telemetry.update();
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

        motorRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRight1.setPower(-power);
        motorRight2.setPower(-power);
        motorLeft1.setPower(power);
        motorLeft2.setPower(power);

        while (motorLeft1.isBusy() & motorLeft2.isBusy() & motorRight1.isBusy() & motorRight2.isBusy()) {
            //wait
        }
        reset();
    }

    public void moveForwardTime(double power, long time) {
        motorRight1.setPower(power);
        motorRight2.setPower(power);
        motorLeft1.setPower(power);
        motorLeft2.setPower(power);
        sleep(time);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
    }
    //  Spins the glyph pusher arm to knock glyphs down the Cryptobox
    public void hitTheGlyph() {
        glyphPusherArm.setPower(1);
        sleep(500);
        glyphPusherArm.setPower(0);
    }
    //  Sets power to 0
    public void reset() {
        motorRight1.setPower(0);
        motorRight2.setPower(0);
        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
    }

    //Moves forward while using Vuforia to detect the Vumark
    public void moveWhileLooking(VuforiaTrackable relicTemplate, VuforiaTrackables relicTrackables, RelicRecoveryVuMark vuMark) {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.addData("cryptobox", cryptobox);
            cryptobox = vuMark;
            relicTrackables.deactivate();
        } else {
            telemetry.addData("Vumark", "not visible");
        }
        telemetry.update();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //  Initialization Code
        //      Drive Motors
        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");

        //      Glyph Track Motors
        glyphTrackRight = hardwareMap.dcMotor.get("glyphTrackRight");
        glyphTrackLeft = hardwareMap.dcMotor.get("glyphTrackLeft");

        glyphPusherArm = hardwareMap.dcMotor.get("glyphPusherArm");
        relicArm = hardwareMap.dcMotor.get("relicArm");

        colorSensor = hardwareMap.colorSensor.get("color");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color");
        //      Servo for raising and lowering the arm for hitting jewels
        ballArm = hardwareMap.servo.get("armServo");
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu.initialize(parameters);

        //      Brake - when the motor is set to 0 it won't roll freely
        motorLeft1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //      Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AapvzVP/////AAAAGWsKXrOLD0tWteq3phY9r7cXWJ30b1s932XkT24qSESw67Xd9qt/HSvf3FEV48IpEjL1fAzYAyllzuGOwJDYNpx0ooH9RRnQkCAv76LChHFvDBPzXBLIedafXHBGT9qDyEy8Vqsq4F4FxyH6cBx0TTFuQksJdIFNTVTtU7Dt6R3qv6H6JXQjBH0WEa1jN3iND3KpSd3oF1dTv9HeHEYxn1/rRiGbneRj3g1kTnyV0cgb+/bipQ4/FBiKuobL778d65Mu+/mzR1cAQA6juUywXSfI1Pyx58vrg8wkTeVlGpAUr+a9htROUaH538h1pdSBlsHobktqihXB69fhSVY2+NlalqDxEEONNuCFDRhJ8RjM";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();

        //      Reversing Motors
        motorLeft1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft2.setDirection(DcMotorSimple.Direction.REVERSE);

        ballArm.setPosition(0.75);
        AutoTransitioner.transitionOnStop(this, "RR_MainTeleop");
//Autonomous Code
        waitForStart();
        while (opModeIsActive()) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            /*hitBall();
            sleep(2000);
            moveWhileLooking(relicTemplate, relicTrackables, vuMark);
            goToCryptobox(0.25, cryptobox);
            turnLeftWithEncoders(0.25);
            moveForward(200, 0.25);
            turnAroundWithEncoders(0.25);
            initialRunGlyphTrack(-1);
            hitTheGlyph();
            sleep(300);
            //  The Robot goes back for more glyphs
            //      if this takes too long it simply goes to the safe zone
            int x = 1;
            while (x < 4 || getRuntime() < 28)
            {
                moveForward(1000, 0.25);
                runGlyphTrack(1, 1000);
                moveForward(-1000, 0.25);
                runGlyphTrack(1, 1000);
                hitTheGlyph();
                x++;
            }
            moveForward(100, 0.25);*/
            sleep(1000);
            stop();
        }

    }
}