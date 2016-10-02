package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Troy on 10/01/16.
  This works

 */
//LinearOpMode

@Autonomous(name = "autonomousMDR", group = "MDR")

public class AutonomousMDR extends LinearOpMode{

    HardwarePushbotTDR         robot   = new HardwarePushbotTDR();
    public DcMotor motorR;
    public DcMotor motorL;
    private ElapsedTime runtime = new ElapsedTime();
    public ColorSensor colsensor;
/*
    public AutonomousMDR(){

    }
*/
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        colsensor = hardwareMap.colorSensor.get("colsensor");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.addData("sensorColor:", colsensor.blue());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // set Speed
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


        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);



    }
}