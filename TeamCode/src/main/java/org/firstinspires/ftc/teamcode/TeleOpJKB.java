package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PWMOutput;
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
    double vl = 1;
    double vr = 1;
    int direction = 1;
    int shot = 0;
    double step = 0;

/*
    public AutonomousMDR(){

    }
*/
    @Override
    public void init() {

        robot.init(hardwareMap);
        robot.liftservo.setPosition(1);
        robot.shotFeeder.setPosition(.9);
        robot.pressservo.setPosition(0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

    }

    @Override
    public void loop(){
        telemetry.addData("Loop", "Running");

        if(gamepad1.right_stick_button){
            direction = 1;//forward to push buttons
        }

        if(gamepad1.left_stick_button){
            direction = -1;//forward to shoot
        }

        double l;
        double r;


        l = gamepad1.left_stick_y;
        r = gamepad1.right_stick_y;

        telemetry.addData("l value:", l);
        telemetry.addData("r value:", r);

        /*//left wheel
        if (l <-0.05 || l > 0.05){
            if(direction == 1){
                robot.MotorL.setPower(l*vl*direction);
            }
            else if (direction ==-1){
                robot.MotorR.setPower(l*vr*direction);
            }
        }
        else{
            if(direction == 1){
                robot.MotorL.setPower(0);
            }
            else if (direction ==-1){
                robot.MotorR.setPower(0);
            }
        }

        //right wheel
        if (r <-0.05 || r > 0.05){
            if(direction == 1){
                robot.MotorR.setPower(r*vr*direction);
            }
            else if (direction ==-1){
                robot.MotorL.setPower(r*vl*direction);
            }
        }
        else {
            if(direction == 1){
                robot.MotorR.setPower(0);
            }
            else if (direction ==-1){
                robot.MotorL.setPower(0);
            }
        }*/







        //shooter

        if(gamepad2.y){//on
            shot = 1;
        }
        if(gamepad2.a){//off
            shot = 0;
        }
        if(shot == 1){
            if(step < 0.95){
                step += 0.01;
            }
            if(step > 0.95){
                step = 1;
            }
        }
        if(shot == 0){
            if(step > 0.05 ){
                step -= 0.01;
            }
            if(step < 0.05){
                step = 0;
            }

        }
        robot.MotorL.setPower(step);
        telemetry.addData("shot", shot);
        telemetry.addData("step", step);
        //robot.ShooterDown.setPower(step);
        //robot.ShooterUp.setPower(-step);


        if(gamepad2.right_bumper){
            runtime.reset();
            while(runtime.seconds() < .8){
                robot.shotFeeder.setPosition(0);
            }
            robot.shotFeeder.setPosition(.9);
        }





        //Conveyor
        if(gamepad2.right_trigger > .5){
            robot.Conveyor.setPower(.7);
        }

        else{
            robot.Conveyor.setPower(0);
        }





        //Lift
        double h = gamepad2.left_stick_y;
        if(h > 0.05 || h < -0.05){
            robot.Lift.setPower(h);
        }
        else{
            robot.Lift.setPower(0);
        }





        //Liftservo up
        if(gamepad2.left_bumper){
            robot.liftservo.setPosition(0);
        }
        if(gamepad2.left_trigger > .5){
            robot.liftservo.setPosition(1);
        }





        //paddle to left
        if(gamepad2.dpad_left){
            robot.pressservo.setPosition(.42);
        }

        //paddle to right
        if(gamepad2.dpad_right){
            robot.pressservo.setPosition(.88);
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