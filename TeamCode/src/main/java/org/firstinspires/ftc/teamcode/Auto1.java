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

public class Auto1 extends LinearOpMode{

    HardwarePushbotTDR         robot   = new HardwarePushbotTDR();
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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //back into ball
        step = 1;
        robot.MotorR.setPower(-.3*vr);
        robot.MotorL.setPower(-.3*vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 3200) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR + 3200 - currentPos", startPosR + 3200 - robot.MotorR.getCurrentPosition() );
            telemetry.update();
            //idle();
        }


        //wait
        step = 2;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .5) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            //idle();
        }


        //turn
        step = 3;
        robot.MotorL.setPower(.7*vl);
        robot.MotorR.setPower(-.7*vr);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 2700) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR + 2700 - currentPos", startPosR + 2700 - robot.MotorR.getCurrentPosition() );
            telemetry.update();
            //idle();
        }

        //wait
        step = 4;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .5) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //forward until white line
        step =5;
        robot.MotorR.setPower(.5*vr);
        robot.MotorL.setPower(.5*vl);
        while (opModeIsActive() && robot.colsensor.blue() < 10){
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
        while (opModeIsActive() && runtime.seconds() < 3) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("tsensor.isPressed()", robot.tsensor.isPressed());
            telemetry.update();
            idle();
        }

        step = 7;
        while (opModeIsActive() && !robot.tsensor.isPressed() ) {
            telemetry.addData("Step:", step);
            if (robot.colsensor.blue() > 10) {
                robot.MotorR.setPower(.3 * vr);
                robot.MotorL.setPower(.1 * vl);
            }

            else if (robot.colsensor.blue() < 10) {
                robot.MotorR.setPower(.1 * vr);
                robot.MotorL.setPower(.3 * vl);
            }


        }

        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);


    }
}