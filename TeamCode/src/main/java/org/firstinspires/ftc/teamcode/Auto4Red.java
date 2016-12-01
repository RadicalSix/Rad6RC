package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Troy on 10/01/16.
  This works

 */
//LinearOpMode

@Autonomous(name = "auto4", group = "Auto")

public class Auto4Red extends LinearOpMode {

    HardwarePushbotTDR robot = new HardwarePushbotTDR();
    VuforiaOp camera = new VuforiaOp();
    private ElapsedTime runtime = new ElapsedTime();
    Boolean beaconOneRed;

    double vl = 1;
    double vr = 1;
    int step = 0;
    double shot = 0;
    String status = "Start";


    /*
        public AutonomousMDR(){

        }
    */
    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);

        double startPosR = robot.MotorR.getCurrentPosition();
        robot.liftservo.setPosition(.0);
        robot.shotFeeder.setPosition(.9);
        robot.pressservo.setPosition(.0);
        robot.conveyorservo.setPosition(0);//in

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        status = "Start, move servo";
        /*telemetry.addData("Status:", status);
        telemetry.update();
        robot.pressservo.setPosition(.36);

        status = "start shooter";
        shot = 0;
        while(opModeIsActive() && shot < 0.4) {
            shot += 0.02;
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.1) {
            shot = .4;
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        status = "shoot first ball";
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 1.5) {
            shot = .4;
            robot.shotFeeder.setPosition(0);
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        status = "feed second ball";
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 1){
            shot = .4;
            robot.shotFeeder.setPosition(.9);
            robot.Conveyor.setPower(.7);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        status = "shoot second ball";
        robot.Conveyor.setPower(0);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 2.5) {
            shot = .4;
            robot.shotFeeder.setPosition(0);
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        while(opModeIsActive() && shot > 0.05) {
            shot -= 0.01;
            robot.shotFeeder.setPosition(.9);
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        status = "drive off wall";
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 1300) {
            telemetry.addData("MotorR units to go", robot.MotorR.getCurrentPosition() - startPosR - 1300);
            telemetry.addData("MotorR current", robot.MotorR.getCurrentPosition());
            telemetry.addData("Status:", status);
            telemetry.update();
            robot.MotorL.setPower(vl * -.5);
            robot.MotorR.setPower(vr * -.5);
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "turn to white line";
        startPosR = robot.MotorR.getCurrentPosition();
        while(opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 1200){
            telemetry.addData("MotorR units to go", robot.MotorR.getCurrentPosition() - startPosR + 1200);
            telemetry.addData("MotorR current", robot.MotorR.getCurrentPosition());
            telemetry.addData("Status:", status);
            telemetry.update();
            robot.MotorL.setPower(vl * .5);
            robot.MotorR.setPower(vr * -.5);
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
*/ //start tomorrow with just the driving
        status = "drive until white line";
        while (opModeIsActive() && robot.colsensor.blue() < 8) {//changed from 6 to 10 10/16
            robot.MotorR.setPower(.55 * vr);
            robot.MotorL.setPower(.55 * vl);
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.addData("Status:", status);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "forward past line";
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(.3 * vr);
        robot.MotorL.setPower(.3 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 100) {
            telemetry.addData("Status:", status);
            telemetry.addData("MotorR units to go", robot.MotorR.getCurrentPosition() - startPosR + 100);
            telemetry.addData("MotorR current", robot.MotorR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        status = "wait";
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .5) {
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        status = "turn until white line";
        robot.MotorR.setPower(.5 * vr);
        robot.MotorL.setPower(-.5 * vl);
        while (opModeIsActive() && robot.colsensor.blue() < 8) {//changed from 6 to 10 10/16
            telemetry.addData("Status:", status);
            telemetry.update();
            idle();
        }

        status = "wait";
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        status = "line follow";
        runtime.reset();
        while (opModeIsActive() && !robot.tsensor.isPressed()) {
            telemetry.addData("Status:", status);
            telemetry.addData("tsensor.isPressed()", robot.tsensor.isPressed());
            telemetry.update();
            if (robot.colsensor.blue() < 6) {//grey
                robot.MotorR.setPower(.5 * vr);
                robot.MotorL.setPower(.1 * vl);
            } else if (robot.colsensor.blue() > 6) {//white
                robot.MotorR.setPower(.1 * vr);
                robot.MotorL.setPower(.5 * vl);
            }
        }

        status = "sense color";
            telemetry.update();
            robot.MotorL.setPower(0);
            robot.MotorR.setPower(0);
            startPosR = robot.MotorR.getCurrentPosition();
            runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.addData("sensorColor:", robot.colsensor.blue());
                telemetry.update();
                if (robot.fruitysensor.blue() > robot.fruitysensor.red()) {
                    beaconOneRed = false;
                }
                else {
                    beaconOneRed = true;
                }
            }

            status = "back up";
            telemetry.update();
            startPosR = robot.MotorR.getCurrentPosition();
            robot.MotorR.setPower(-.4 * vr);
            robot.MotorL.setPower(-.4 * vl);
            while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 150) {
                telemetry.addData("MotorR to go", robot.MotorR.getCurrentPosition() - startPosR - 150);
                telemetry.addData("Status:", status);
                telemetry.update();
            }

            status = "turn paddles";
            telemetry.update();
            robot.MotorL.setPower(0);
            robot.MotorR.setPower(0);
            startPosR = robot.MotorR.getCurrentPosition();
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1) {
                telemetry.addData("Status:", status);
                telemetry.update();
                if (beaconOneRed) {
                    robot.pressservo.setPosition(.93);
                }
                else {
                    robot.pressservo.setPosition(.36);
                }
            }

            status = "forward and press buttons";
            telemetry.update();
            startPosR = robot.MotorR.getCurrentPosition();
            robot.MotorR.setPower(.4 * vr);
            robot.MotorL.setPower(.4 * vl);
            while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 300) {
                telemetry.addData("currentPos - startPosR + 400", robot.MotorR.getCurrentPosition() - startPosR + 400);
                telemetry.addData("Status:", status);
                telemetry.update();
            }






















/*

            //back into ball
        step = 2;
        robot.MotorR.setPower(-.6*vr);
        robot.MotorL.setPower(-.6*vl);
        shot = 0;
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 2000) {
        robot.ShooterDown.setPower(shot);
        robot.ShooterUp.setPower(-shot);
            telemetry.addData("Step:", step);
            telemetry.addData("currentPos - startPosR + 2000", robot.MotorR.getCurrentPosition() - startPosR + 2000);
            telemetry.update();
            //idle();
        }


        //wait
        step = 3;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            //idle();
        }


        //turn towards white line, knocking ball off
        step = 4;
        robot.MotorL.setPower(1*vl);
        robot.MotorR.setPower(-1*vr);
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 1600) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("currentPos - startPosR + 1600 - ", robot.MotorR.getCurrentPosition() - startPosR + 1600);
            telemetry.update();
            //idle();
        }

        //wait
        step = 5;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            //idle();
        }

        //drive towards middle of first picture until -300 away
        step = 6;
        boolean done = false;
        boolean found = false;
        while (opModeIsActive() && !done) {
            telemetry.addData("Step:", step);
            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    y = translation.get(0);
                    //up down in terms of the pic when phone is on side
                    x = translation.get(1);
                    //left right in terms of the pic when phone is on side
                    z = translation.get(2);
                    //distance away from the pic
                    telemetry.addData(beac.getName() + "-Translation", translation);
                    degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                    telemetry.addData(beac.getName() + "-Zval", z);
                    telemetry.addData(beac.getName() + "-Yval", y);
                    telemetry.addData(beac.getName() + "-Xval", x);
                }
            }

            if(z < 0){
                found = true;
            }

            if(found) {
                if (x > 0) {
                    robot.MotorL.setPower(1 * vl);
                    robot.MotorR.setPower(.5 * vr);
                    telemetry.addData("right", x);
                } else if (x < 0) {
                    robot.MotorL.setPower(.5 * vl);
                    robot.MotorR.setPower(1 * vr);
                    telemetry.addData("left", x);
                } /*else {//didnt turn enough
                    robot.MotorL.setPower(.7 * vl);
                    robot.MotorR.setPower(-.7 * vr);
                }
                if (z > -300) {
                    done = true;
                }
            }

            telemetry.addData("found:", found);
            telemetry.addData("done:", done);
            telemetry.update();
        }

        //wait
        step = 7;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //turn until square with first picture
        step = 8;
        boolean done2 = false;
        found = false;
        while (opModeIsActive() && !done2) {
            telemetry.addData("Step:", step);
            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    y = translation.get(0);
                    //up down in terms of the pic when phone is on side
                    x = translation.get(1);
                    //left right in terms of the pic when phone is on side
                    z = translation.get(2);
                    //distance away from the pic
                    telemetry.addData(beac.getName() + "-Translation", translation);
                    degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                    telemetry.addData(beac.getName() + "-Zval", z);
                    telemetry.addData(beac.getName() + "-Yval", y);
                    telemetry.addData(beac.getName() + "-Xval", x);
                }
            }

                if(z < 0){
                    found = true;
                }

                if(found) {
                    if (degreesToTurn > -179 && degreesToTurn < -90) {
                        robot.MotorL.setPower(-1 * vl);
                        robot.MotorR.setPower(1 * vr);
                        telemetry.addData("right", degreesToTurn);
                    } else if (degreesToTurn < 179 && degreesToTurn > 90) {
                        robot.MotorL.setPower(1 * vl);
                        robot.MotorR.setPower(-1 * vr);
                        telemetry.addData("left", degreesToTurn);
                    }
                }


            if ((degreesToTurn < -170 && degreesToTurn > -181) || (degreesToTurn > 170 && degreesToTurn < 181)){
                done2 = true;
            }
            telemetry.update();
        }

        //wait
        step = 9;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //turn 90 degrees to right
        step = 10;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(-1 * vr);
        robot.MotorL.setPower(1 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 1700) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("currentPos - startPosR + 1700", robot.MotorR.getCurrentPosition() - startPosR + 1700);
            telemetry.update();
            idle();
        }

        //wait
        step = 11;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //forward until white line
        step = 12;
        robot.MotorR.setPower(.7 * vr);
        robot.MotorL.setPower(.7 * vl);
        while (opModeIsActive() && robot.colsensor.blue() < 6) {//changed from 6 to 10 10/16
            telemetry.addData("Step:", step);
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }
        //wait
        step = 13;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        //turn to orient more towards beacon
        step = 14;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(.5 * vr);
        robot.MotorL.setPower(-.5 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 1200) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("currentPos - startPosR - 1200", robot.MotorR.getCurrentPosition() - startPosR - 1200);
            telemetry.update();
            idle();
        }

        //wait
        step = 15;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        step = 16;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4) {
            telemetry.addData("Step:", step);
            for(VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                if(pose != null){
                    VectorF translation = pose.getTranslation();
                    y = translation.get(0);//up down in terms of the pic when phone is on side
                    x = translation.get(1);//left right in terms of the pic when phone is on side
                    z = translation.get(2);//distance away from the pic
                    telemetry.addData(beac.getName() + "-Translation", translation);
                    degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                    telemetry.addData(beac.getName() + "-Zval", z);
                    telemetry.addData(beac.getName() + "-Yval", y);
                    telemetry.addData(beac.getName() + "-Xval", x);
                }
            }
        }


        //use vuforia to find beacon
        step = 17;
        boolean done20 = false;
        while (opModeIsActive() && z < -200) {
            telemetry.addData("Step:", step);
            for(VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                if(pose != null){
                    VectorF translation = pose.getTranslation();
                    y = translation.get(0);//up down in terms of the pic when phone is on side
                    x = translation.get(1);//left right in terms of the pic when phone is on side
                    z = translation.get(2);//distance away from the pic
                    telemetry.addData(beac.getName() + "-Translation", translation);
                    degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                    telemetry.addData(beac.getName() + "-Zval", z);
                    telemetry.addData(beac.getName() + "-Yval", y);
                    telemetry.addData(beac.getName() + "-Xval", x);
                }
                if (x > 0){
                    robot.MotorL.setPower(.9 *vl);
                    robot.MotorR.setPower(-.2 *vr);
                    telemetry.addData("right", x);
                }
                if (x < 0){
                    robot.MotorL.setPower(-.2 *vl);
                    robot.MotorR.setPower(.7 *vr);
                    telemetry.addData("left", x);
                }
                //if(z > -200){
                 //   done20 = true;
                //}
                telemetry.addData("done20", done20);
            }
            telemetry.update();
        }

        //wait
        step = 18;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            telemetry.addData("Step:", step);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        step =19;
        while(opModeIsActive()) {
            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    y = translation.get(0);//up down in terms of the pic when phone is on side
                    x = translation.get(1);//left right in terms of the pic when phone is on side
                    z = translation.get(2);//distance away from the pic
                    telemetry.addData(beac.getName() + "-Translation", translation);
                    degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                    telemetry.addData(beac.getName() + "-Zval", z);
                    telemetry.addData(beac.getName() + "-Yval", y);
                    telemetry.addData(beac.getName() + "-Xval", x);
                }
            }
            if (degreesToTurn > 0 && degreesToTurn < 178) {
                while (degreesToTurn < 178) {
                    robot.MotorL.setPower(.7 * vl);
                    robot.MotorR.setPower(-.7 * vr);
                }
            } else if (degreesToTurn < 0 && degreesToTurn > -178) {
                while (degreesToTurn > -178) {
                    robot.MotorL.setPower(-.7 * vl);
                    robot.MotorR.setPower(.7 * vr);
                }
            }
        }

        step = 20;
            while(opModeIsActive() && robot.tsensor.isPressed()){
                robot.MotorL.setPower(.5 * vl);
                robot.MotorR.setPower(.5 * vr);
            }

        //wait
        step = 21;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();

            if (robot.fruitysensor.blue() > robot.fruitysensor.red()) {
                beaconOneRed = false;

            }
            else {
                beaconOneRed = true;
            }
            idle();
        }

        //back up
        step = 22;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(-.4 * vr);
        robot.MotorL.setPower(-.4 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 150) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("currentPos - startPosR + 150", robot.MotorR.getCurrentPosition() - startPosR + 150);
            telemetry.update();
            idle();
        }

        //press appropriate beacon button
        step = 23;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4) {
            telemetry.addData("Step:", step);
            telemetry.update();
            if (beaconOneRed) {
                robot.pressservo.setPosition(.93);

            }
            else {
                robot.pressservo.setPosition(.36);
            }

            idle();
        }


        //back up
        step = 24;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(.4 * vr);
        robot.MotorL.setPower(.4 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 160) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("currentPos - startPosR - 160", robot.MotorR.getCurrentPosition() - startPosR - 160);
            telemetry.update();
            idle();
        }

        //wait
        step = 25;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //back up
        step = 26;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(-.3 * vr);
        robot.MotorL.setPower(-.3 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 650) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR + 650 - currentPos", startPosR + 650 - robot.MotorR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        //wait, reset servo
        step = 27;
        robot.pressservo.setPosition(.4);
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //turn
        step = 28;
        robot.MotorL.setPower(-.7*vl);
        robot.MotorR.setPower(.7*vr);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 1050) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR - 1050 - currentPos", startPosR - 1050 - robot.MotorR.getCurrentPosition() );
            telemetry.update();
            //idle();
        }

        //wait
        step = 29;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //forward off of white line
        step = 30;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(.3 * vr);
        robot.MotorL.setPower(.3 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 800) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR - 200 - currentPos", startPosR - 800 - robot.MotorR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        //forward until next white line
        step = 31;
        robot.MotorR.setPower(.5 * vr);
        robot.MotorL.setPower(.5 * vl);
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.colsensor.blue() < 6 ) {
            telemetry.addData("Step:", step);
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //wait
        step = 32;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //forward off of line
        step = 33;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(.3 * vr);
        robot.MotorL.setPower(.3 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 170) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR - 170 - currentPos", startPosR - 170 - robot.MotorR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        //turn to orient towards beacon
        step = 34;
        robot.MotorL.setPower(.3*vl);
        robot.MotorR.setPower(-.3*vr);
        while (opModeIsActive() && robot.colsensor.blue() < 6) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            //idle();
        }

        //follow white line to the beacon
        step = 35;
        runtime.reset();
        while (opModeIsActive() && !robot.tsensor.isPressed()) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("tsensor.isPressed()", robot.tsensor.isPressed());
            telemetry.update();


            if (robot.colsensor.blue() < 6) {//grey

                robot.MotorR.setPower(.0 * vr);
                robot.MotorL.setPower(.6 * vl);
            } else if (robot.colsensor.blue() > 6) {//white
                robot.MotorR.setPower(.6 * vr);
                robot.MotorL.setPower(.0 * vl);
            }


        }

        //press appropriate beacon button
        step = 36;
        if (robot.fruitysensor.blue() > robot.fruitysensor.red()) {
            robot.pressservo.setPosition(.9);

        }
        else {
            robot.pressservo.setPosition(.0);
        }

        //wait
        step = 37;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }

        //back up to park on center vortex
        step = 38;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(-.3*vr);
        robot.MotorL.setPower(-.3*vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 3800) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("startPosR + 3200 - currentPos", startPosR + 3800 - robot.MotorR.getCurrentPosition() );
            telemetry.update();
            //idle();
        }

        //wait
        step = 39;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .2) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
            */
        }


    }



