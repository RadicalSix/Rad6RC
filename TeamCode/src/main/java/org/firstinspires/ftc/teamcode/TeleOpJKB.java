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
    double shotspeed = 1;
    double startPosR;
    boolean backdone = false;
    double reduceSpeed = 1;

/*
    public AutonomousMDR(){

    }
*/
    @Override
    public void init() {

        robot.init(hardwareMap);
        robot.liftservo.setPosition(.25);
        robot.shotFeeder.setPosition(.9);
        robot.pressservo.setPosition(0);
        robot.conveyorservo.setPosition(0);//in

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

    }

    @Override
    public void loop(){
        telemetry.addData("Loop", "Running");

        if(gamepad1.b){
            reduceSpeed = 1;
        }
        if(gamepad1.x){
            reduceSpeed = .5;
        }
        telemetry.addData("reduceSpeed", reduceSpeed);


        if(gamepad1.right_stick_button){
            direction = -1;//forward to push buttons
            telemetry.addData("Buttons Direction", direction);
        }



        if(gamepad1.left_stick_button){
            direction = 1;//forward to shoot, lift
            telemetry.addData("Lift Direction", direction);
        }

        double l;
        double r;


        l = -gamepad1.left_stick_y;
        r = -gamepad1.right_stick_y;

        telemetry.addData("l value:", l);
        telemetry.addData("r value:", r);

        //left wheel
        if (l <-0.05 || l > 0.05){
            if(direction == 1){
                robot.MotorR.setPower(l*vr*direction*reduceSpeed);
            }
            else if (direction ==-1){
                robot.MotorL.setPower(l*vl*direction*reduceSpeed);
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
                robot.MotorL.setPower(r*vl*direction*reduceSpeed);
            }
            else if (direction ==-1){
                robot.MotorR.setPower(r*vr*direction*reduceSpeed);
            }
        }
        else {
            if(direction == 1){
                robot.MotorR.setPower(0);
            }
            else if (direction ==-1){
                robot.MotorL.setPower(0);
            }
        }







        //shooter

        if(gamepad1.a){
            shotspeed = .37;
        }
        if(gamepad1.y){
            shotspeed = .44;
        }


        if(gamepad2.y){//on
            shot = 1;
        }
        if(gamepad2.a){//off
            shot = 0;
        }
        if(shot == 1){
            if(step < (shotspeed - 0.05)){
                step += 0.02;
            }
            if(step > (shotspeed - 0.05)){
                step = shotspeed;
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

        telemetry.addData("shot", shot);
        telemetry.addData("step", step);
        telemetry.addData("shotspeed", shotspeed);
        robot.ShooterDown.setPower(step);
        robot.ShooterUp.setPower(-step);


        if(gamepad2.right_bumper){
            runtime.reset();
            while(runtime.seconds() < .8){
                robot.shotFeeder.setPosition(0);
            }
            robot.shotFeeder.setPosition(.9);
        }

        if(gamepad1.dpad_down){
            startPosR = robot.MotorR.getCurrentPosition();
            backdone = false;
            while((robot.MotorR.getCurrentPosition() > startPosR - 2000) && !backdone){
                robot.MotorR.setPower(vr*.4);
                robot.MotorL.setPower(vr*.4);
                telemetry.addData("encoder units left", robot.MotorR.getCurrentPosition() - startPosR + 2000);
                telemetry.update();
                if(gamepad1.dpad_up){
                    backdone = true;
                }
            }
            robot.MotorR.setPower(0);
            robot.MotorL.setPower(0);
        }



        //Conveyor
        if(gamepad2.right_trigger > .5){
            robot.Conveyor.setPower(.7);
        }

        else{
            robot.Conveyor.setPower(0);
        }

        if(gamepad2.x){
            robot.conveyorservo.setPosition(1);//out
        }
        if(gamepad2.b){
            robot.conveyorservo.setPosition(0);//in
        }





        //Lift
        double h = -gamepad2.left_stick_y;
        telemetry.addData("h", h);
        if(((h > 0.05) || (h < -0.05)) && (robot.liftservo.getPosition() == .95)){
            robot.Lift.setPower(h);
        }
        else{
            robot.Lift.setPower(0);
        }





        //Liftservo up
        if(gamepad2.left_bumper){
            robot.liftservo.setPosition(.95);
        }
        if(gamepad2.left_trigger > .5){
            robot.liftservo.setPosition(.25);//down
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
        /*robot.pressservo.setPosition(.88);
        //slow it down
        boolean done3 = false;
        while(!done3) {
            if (step > 0.05) {
                step -= 0.01;
            }
            if (step < 0.05) {
                step = 0;
                done3 = true;
            }
            telemetry.addData("step", step);
            robot.ShooterDown.setPower(step);
            robot.ShooterUp.setPower(-step);
        }*/
    }


}