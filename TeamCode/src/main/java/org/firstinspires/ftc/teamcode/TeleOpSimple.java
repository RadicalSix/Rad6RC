package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Troy on 10/01/16.
  This works

 */
//LinearOpMode

//@TeleOp(name = "TeleOpSimple", group = "JKB")

public class TeleOpSimple extends OpMode{

    HardwarePushbotSimple         robot   = new HardwarePushbotSimple();
    private ElapsedTime runtime = new ElapsedTime();

/*
    public AutonomousMDR(){

    }
*/
    @Override
    public void init() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

    }

    @Override
    public void loop(){
        telemetry.addData("Loop", "Running");
        double l;
        double r;


        l = -gamepad1.left_stick_y;
        r = -gamepad1.right_stick_y;

        telemetry.addData("l value:", l);
        telemetry.addData("r value:", r);

        //left wheel
        if (l <-0.05 || l > 0.05){
            robot.MotorL.setPower(l);
        }
        else{
            robot.MotorL.setPower(0);
        }

        //right wheel
        if (r <-0.05 || r > 0.05){
            robot.MotorR.setPower(r);
        }
        else {
            robot.MotorR.setPower(0);
        }




        //paddle to left
        if(gamepad2.dpad_left){
            robot.pressservo.setPosition(.36);
        }

        //paddle to right
        if(gamepad2.dpad_right){
            robot.pressservo.setPosition(.93);
        }

        //paddle to initial
        if(gamepad2.dpad_down){
            robot.pressservo.setPosition(0);
        }


    }

    @Override
    public void stop() {
    }


}