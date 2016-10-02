package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Troy on 10/01/16.
  This works

 */
//LinearOpMode

@TeleOp(name = "TeleOpJKB", group = "JKB")

public class TeleOpJKB extends OpMode{

    HardwarePushbotTDR         robot   = new HardwarePushbotTDR();
    public DcMotor motorR;
    public DcMotor motorL;
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

        if (l <-0.05 || l > 0.05){
            robot.MotorL.setPower(l);
        }
        else{
            robot.MotorL.setPower(0);
        }

        if (r <-0.05 || r > 0.05){
            robot.MotorR.setPower(r);
        }
        else {
            robot.MotorR.setPower(0);
        }




    }

    @Override
    public void stop() {
    }


}