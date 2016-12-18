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

@Autonomous(name = "Info", group = "Auto")

public class Info extends LinearOpMode {

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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        while (opModeIsActive() ) {
            telemetry.addData("tsensor.isPressed()", robot.tsensor.isPressed());
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("currentPosPressservo", robot.pressservoR.getPosition());
            telemetry.addData("colsensor, blue:", robot.colsensor.blue());
            telemetry.addData("colsensor, green:", robot.colsensor.green());
            telemetry.addData("colsensor, red:", robot.colsensor.red());
            telemetry.addData("fruitysensor, blue:", robot.fruitysensor.blue());
            telemetry.addData("fruitysensor, red:", robot.fruitysensor.red());
            telemetry.addData("fruitysensor, green:", robot.fruitysensor.green());
            telemetry.update();
            //idle();
        }





    }
}

