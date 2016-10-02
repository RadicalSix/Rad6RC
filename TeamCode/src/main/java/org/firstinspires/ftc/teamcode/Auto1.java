package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Troy on 10/01/16.
  This works

 */
//LinearOpMode

@Autonomous(name = "autonomousJKB", group = "JKB")

public class Auto1 extends LinearOpMode{

    HardwarePushbotTDR         robot   = new HardwarePushbotTDR();
    public DcMotor motorR;
    public DcMotor motorL;
    private ElapsedTime runtime = new ElapsedTime();

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

        // run for 3 seconds
        runtime.reset();
        while (robot.MotorR.getCurrentPosition() < startPosR + 3800) {
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.update();
            //idle();
        }

        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);




    }
}