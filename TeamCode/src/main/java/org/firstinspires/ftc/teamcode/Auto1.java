package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Troy on 10/01/16.
  This works

 */
//LinearOpMode

@Autonomous(name = "auto1", group = "Auto")

public class Auto1 extends LinearOpMode {

    HardwarePushbotTDR robot = new HardwarePushbotTDR();
    public DcMotor motorR;
    public DcMotor motorL;
    private ElapsedTime runtime = new ElapsedTime();
    public ColorSensor colsensor;

    double vl = 0.75;
    double vr = 1.0;
    int step = 0;


    /*
        public AutonomousMDR(){

        }
    */
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        double startPosR = robot.MotorR.getCurrentPosition();
        robot.pressservo.setPosition(.4);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Oct 16- start robot with phone on/off button almost touching wall, black zip tie on beam above right motor in line with left side of floor mat ridge


        //back into ball
        step = 1;
        robot.MotorR.setPower(-.3*vr);
        robot.MotorL.setPower(-.3*vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 3400) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR + 3200 - currentPos", startPosR + 3400 - robot.MotorR.getCurrentPosition() );
            telemetry.update();
            //idle();
        }


        //wait
        step = 2;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            //idle();
        }


        //turn towards white line, knocking ball off
        step = 3;
        robot.MotorL.setPower(.7*vl);
        robot.MotorR.setPower(-.7*vr);
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 2300) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR + 2850 - currentPos", startPosR + 2300 - robot.MotorR.getCurrentPosition() );
            telemetry.update();
            //idle();
        }

        //wait
        step = 4;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //forward until white line
        step = 5;
        robot.MotorR.setPower(.5 * vr);
        robot.MotorL.setPower(.5 * vl);
        while (opModeIsActive() && robot.colsensor.blue() < 6) {//changed from 6 to 10 10/16
            telemetry.addData("Step:", step);
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }
        //wait
        step = 6;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        //forward for a bit
        /*step = 7;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(.3 * vr);
        robot.MotorL.setPower(.3 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 10) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR - 10 - currentPos", startPosR - 10 - robot.MotorR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        //wait
        step = 8;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .5) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.update();
            idle();
        }
*/
        //turn to orient more towards beacon
        step = 9;
        robot.MotorR.setPower(.3 * vr);
        robot.MotorL.setPower(-.4 * vl);
        while (opModeIsActive() && robot.colsensor.blue() < 6) {//changed from 6 to 10 10/16
            telemetry.addData("Step:", step);
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.addData("motorL power:", robot.MotorL.getPower());
            telemetry.addData("motorR power:", robot.MotorR.getPower());
            telemetry.update();
            idle();
        }

        //wait
        step = 10;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //follow white line until beacon
        step = 11;
        runtime.reset();
        while (opModeIsActive() && !robot.tsensor.isPressed()) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("tsensor.isPressed()", robot.tsensor.isPressed());
            telemetry.update();


            if (robot.colsensor.blue() < 6) {//grey

                robot.MotorR.setPower(.6 * vr);
                robot.MotorL.setPower(.0 * vl);
            } else if (robot.colsensor.blue() > 6) {//white
                robot.MotorR.setPower(.0 * vr);
                robot.MotorL.setPower(.6 * vl);
            }


        }

        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        //wait
        step = 12;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //press appropriate beacon button
        step = 13;
        if (robot.fruitysensor.blue() > robot.fruitysensor.red()) {
            robot.pressservo.setPosition(.9);

        }
        else {
            robot.pressservo.setPosition(.0);
        }

        //wait
        step = 14;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //back up
        step = 15;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(-.3 * vr);
        robot.MotorL.setPower(-.3 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 650) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR + 650 - currentPos", startPosR + 650 - robot.MotorR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        //wait, reset servo
        step = 16;
        robot.pressservo.setPosition(.4);
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //turn
        step = 17;
        robot.MotorL.setPower(-.7*vl);
        robot.MotorR.setPower(.7*vr);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 1050) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR - 1050 - currentPos", startPosR - 1050 - robot.MotorR.getCurrentPosition() );
            telemetry.update();
            //idle();
        }

        //wait
        step = 18;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //forward off of white line
        step = 19;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(.3 * vr);
        robot.MotorL.setPower(.3 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 800) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR - 200 - currentPos", startPosR - 800 - robot.MotorR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        //forward until next white line
        step = 20;
        robot.MotorR.setPower(.5 * vr);
        robot.MotorL.setPower(.5 * vl);
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.colsensor.blue() < 6 ) {
            telemetry.addData("Step:", step);
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //wait
        step = 21;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //forward off of line
        step = 22;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(.3 * vr);
        robot.MotorL.setPower(.3 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 170) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR - 170 - currentPos", startPosR - 170 - robot.MotorR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        //turn to orient towards beacon
        step = 23;
        robot.MotorL.setPower(.3*vl);
        robot.MotorR.setPower(-.3*vr);
        while (opModeIsActive() && robot.colsensor.blue() < 6) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            //idle();
        }

        //follow white line to the beacon
        step = 24;
        runtime.reset();
        while (opModeIsActive() && !robot.tsensor.isPressed()) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("tsensor.isPressed()", robot.tsensor.isPressed());
            telemetry.update();


            if (robot.colsensor.blue() < 6) {//grey

                robot.MotorR.setPower(.0 * vr);
                robot.MotorL.setPower(.6 * vl);
            } else if (robot.colsensor.blue() > 6) {//white
                robot.MotorR.setPower(.6 * vr);
                robot.MotorL.setPower(.0 * vl);
            }


        }

        //press appropriate beacon button
        step = 25;
        if (robot.fruitysensor.blue() > robot.fruitysensor.red()) {
            robot.pressservo.setPosition(.9);

        }
        else {
            robot.pressservo.setPosition(.0);
        }

        //wait
        step = 26;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //back up to park on center vortex
        step = 27;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(-.3*vr);
        robot.MotorL.setPower(-.3*vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 3800) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR + 3200 - currentPos", startPosR + 3800 - robot.MotorR.getCurrentPosition() );
            telemetry.update();
            //idle();
        }

        //wait
        step = 28;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }
    }
}
