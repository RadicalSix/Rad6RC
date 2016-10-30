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

@Autonomous(name = "TestTurn", group = "Auto")

public class TestTurn extends LinearOpMode {

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

        while(opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 2400){
            robot.MotorR.setPower(.5 * vr);
            robot.MotorL.setPower(-.5 * vl);

            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("currentPosR - startPosR + 2400", robot.MotorR.getCurrentPosition() - startPosR + 2400);
            telemetry.update();
        }

        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

    }
}