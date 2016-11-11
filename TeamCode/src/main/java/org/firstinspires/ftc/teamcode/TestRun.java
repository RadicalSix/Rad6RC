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

@Autonomous(name = "TestRun", group = "Auto")

public class TestRun extends LinearOpMode{

    HardwarePushbotTDR         robot   = new HardwarePushbotTDR();
    public DcMotor motorR;
    public DcMotor motorL;
    private ElapsedTime runtime = new ElapsedTime();
    public ColorSensor colsensor;

    double vl = 1;
    double vr = 0.5;
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

        robot.MotorR.setPower(.7*vr);
        robot.MotorL.setPower(.7*vl);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 1.8){

        }


        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 1.5){

        }

        robot.MotorR.setPower(-.7*vr);
        robot.MotorL.setPower(-.7*vl);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 1.8){

        }



    }
}