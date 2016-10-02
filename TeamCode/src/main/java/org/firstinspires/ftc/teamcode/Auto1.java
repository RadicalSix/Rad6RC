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

    double vl = 0.98;
    double vr = 1.0;

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

        // set Speed
        robot.MotorR.setPower(-.3*vr);
        robot.MotorL.setPower(-.3*vl);

        // back into ball
        runtime.reset();
        while (robot.MotorR.getCurrentPosition() < startPosR + 3800) {
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.update();
            //idle();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();

        //turn
        robot.MotorL.setPower(.3);
        robot.MotorR.setPower(-.3);
        while (robot.MotorR.getCurrentPosition() > startPosR - 800) {
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.update();
            //idle();
        }

        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();

        //forward until white line
        robot.MotorR.setPower(.5);
        robot.MotorL.setPower(.5);

        // run for 3 seconds
        runtime.reset();
        while (opModeIsActive() && colsensor.blue() < 10)
        {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", colsensor.blue());
            telemetry.update();
            idle();
        }


    }
}