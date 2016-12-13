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

  This works

 */
//LinearOpMode

@Autonomous(name = "auto4Red", group = "Auto")

public class Auto4Red extends LinearOpMode {

    HardwarePushbotTDR robot = new HardwarePushbotTDR();
    VuforiaOp camera = new VuforiaOp();
    private ElapsedTime runtime = new ElapsedTime();
    Boolean beaconOneRed;
    Boolean beaconTwoRed;
    Boolean doneDrive1 = false;
    Boolean beaconOneDone = false;
    Boolean beaconTwoDone = false;
    Boolean forwardTwoDone = false;
    Boolean turnTwoDone = false;
    Boolean longDriveDone = false;
    Boolean followOneDone = false;


    double vl = 1;//change for direction and battery
    double vr = 1;//change for direction and battery
    double shotSpeed = .39;//change for battery
    int step = 0;
    double shot = 0;
    double lastPosR = 0;
    double twoLastPosR = 0;
    int beaconOneCount = 0;
    int BeaconTwoCount = 0;
    int forwardTwoCount = 0;
    int turnTwoCount = 0;
    int followOneCount = 0;
    int turnOneCount = 0;
    String status = "Start";


    /*
        public AutonomousMDR(){

        }
    */
    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);

        double startPosR = robot.MotorR.getCurrentPosition();
        robot.liftservo.setPosition(.25);
        robot.shotFeeder.setPosition(.9);
        robot.conveyorservo.setPosition(0);//in
        robot.pressservo.setPosition(0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //shoot first ball then move servo so it can help hold up the ball

        /*status = "Start, move servo";
        telemetry.addData("Status:", status);
        telemetry.update();


        status = "start shooter";
        shot = 0;
        while(opModeIsActive() && shot < shotSpeed) {
            shot += 0.02;
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5) {
            shot = shotSpeed;
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
            shot = shotSpeed;
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
        while(opModeIsActive() && runtime.seconds() < 2){
            robot.pressservo.setPosition(.36);
            robot.shotFeeder.setPosition(.9);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        status = "shoot second ball";
        robot.Conveyor.setPower(0);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 2.5) {
            shot = shotSpeed;
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

        status = "drive forward off wall";
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 1300) {
            telemetry.addData("MotorR units to go", robot.MotorR.getCurrentPosition() - startPosR - 1300);
            telemetry.addData("MotorR current", robot.MotorR.getCurrentPosition());
            telemetry.addData("Status:", status);
            telemetry.update();
            robot.MotorL.setPower(vl * .5);
            robot.MotorR.setPower(vr * .5);
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "turn to white line";
        startPosR = robot.MotorR.getCurrentPosition();
        while(opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 1500){
            telemetry.addData("MotorR units to go", robot.MotorR.getCurrentPosition() - startPosR - 1500);
            telemetry.addData("MotorR current", robot.MotorR.getCurrentPosition());
            telemetry.addData("Status:", status);
            telemetry.update();
            robot.MotorL.setPower(vl * -.5);
            robot.MotorR.setPower(vr * .5);
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);*/

        startPosR = robot.MotorR.getCurrentPosition();
        status = "drive back until white line";
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 2800 && !doneDrive1) {//stop if hit line or go certain distance
            if(robot.colsensor.blue() > 8){
                doneDrive1 = true;//hit white line
            }
            robot.MotorR.setPower(-.55 * vr);
            robot.MotorL.setPower(-.55 * vl);
            robot.pressservo.setPosition(0);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorR current", robot.MotorR.getCurrentPosition()- startPosR);
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "drive back past line";
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(-.3 * vr);
        robot.MotorL.setPower(-.3 * vl);
        while (opModeIsActive() && (robot.MotorR.getCurrentPosition() > startPosR - 650) && doneDrive1) {//only does this step if hit white line
            telemetry.addData("Status:", status);
            telemetry.addData("MotorR units to go", robot.MotorR.getCurrentPosition() - startPosR + 650);
            telemetry.addData("MotorR current", robot.MotorR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        status = "turn until white line";
        runtime.reset();
        turnOneCount = 0;
        lastPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.colsensor.blue() < 8) {
            telemetry.addData("Status:", status);
            telemetry.update();
            robot.MotorR.setPower(-.33 * vr);
            robot.MotorL.setPower(.33 * vl);
            /*if(lastPosR == robot.MotorR.getCurrentPosition()) {
                turnOneCount++;
            }
            else if(lastPosR != robot.MotorR.getCurrentPosition()) {
                turnOneCount = 0;
            }
            if(turnOneCount > 25){
                while(robot.MotorR.getCurrentPosition() < startPosR + 70) {
                    robot.MotorR.setPower(.25);
                    robot.MotorL.setPower(.25);
                }
                turnOneCount = 0;
            }*/
            if(runtime.seconds() > 1){
                if(lastPosR == robot.MotorR.getCurrentPosition()){
                    while((robot.MotorR.getCurrentPosition() < (startPosR + 600)) && robot.colsensor.blue() < 8) {
                        robot.MotorR.setPower(.5);
                        robot.MotorL.setPower(.5);
                    }
                }
                lastPosR = robot.MotorR.getCurrentPosition();
                runtime.reset();
            }
            //lastPosR = robot.MotorR.getCurrentPosition();
            telemetry.addData("lastPosR", lastPosR);
            telemetry.addData("current", robot.MotorR.getCurrentPosition());
            telemetry.addData("turnOneCount",turnOneCount);
        }

        status = "line follow";
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        lastPosR = robot.MotorR.getCurrentPosition();
        twoLastPosR = 0;
        while (opModeIsActive() && !robot.tsensor.isPressed()) {
            telemetry.addData("Status:", status);
            telemetry.addData("tsensor.isPressed()", robot.tsensor.isPressed());
            telemetry.update();
            if (robot.colsensor.blue() < 8) {//grey
                robot.MotorR.setPower(.2 * vr);
                robot.MotorL.setPower(-.4 * vl);
            } else if (robot.colsensor.blue() > 8) {//white
                robot.MotorR.setPower(-.4 * vr);
                robot.MotorL.setPower(.2 * vl);
            }
            if(runtime.seconds()> 3){
                telemetry.addData("Status:", "Stuck");
                if(Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 10){
                    if(robot.colsensor.blue() > 8){
                        startPosR = robot.MotorR.getCurrentPosition();
                        while(robot.MotorR.getCurrentPosition() < (startPosR + 100 + (followOneCount * 50))) {
                            robot.MotorR.setPower(.5);
                            robot.MotorL.setPower(.25);
                        }
                        robot.MotorR.setPower(0);
                        robot.MotorL.setPower(0);
                    }
                    else if(robot.colsensor.blue() < 8){
                        startPosR = robot.MotorR.getCurrentPosition();
                        while(robot.MotorR.getCurrentPosition() < (startPosR + 250 + (followOneCount * 50))) {
                            robot.MotorR.setPower(.25);
                            robot.MotorL.setPower(.5);
                        }
                        robot.MotorR.setPower(0);
                        robot.MotorL.setPower(0);
                    }
                }
                runtime.reset();
                lastPosR = robot.MotorR.getCurrentPosition();
                followOneCount ++;
            }
            telemetry.addData("Time", runtime.seconds());
            telemetry.addData(" Change in last and current", Math.abs(lastPosR - robot.MotorR.getCurrentPosition()));
            telemetry.addData("followOneCount", followOneCount);
        }

        status = "sense color";
            telemetry.update();
            robot.MotorL.setPower(0);
            robot.MotorR.setPower(0);
            startPosR = robot.MotorR.getCurrentPosition();
            runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .1) {
                telemetry.addData("Status:", status);
                telemetry.addData("sensorColor:", robot.colsensor.blue());
                telemetry.update();
                if (robot.fruitysensor.blue() > robot.fruitysensor.red()) {
                    beaconOneRed = false;
                }
                else {
                    beaconOneRed = true;
                }
            }

            status = "forward off beacon 1";
            telemetry.update();
            startPosR = robot.MotorR.getCurrentPosition();
            robot.MotorR.setPower(.7 * vr);
            robot.MotorL.setPower(.7 * vl);
            while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 225) {
                telemetry.addData("Status:", status);
                telemetry.addData("MotorR to go", robot.MotorR.getCurrentPosition() - startPosR - 225);
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

            status = "backward and press buttons, beacon 1";
            startPosR = robot.MotorR.getCurrentPosition();
            while (opModeIsActive() && (robot.MotorR.getCurrentPosition() > startPosR - 330) && !beaconOneDone) {
                if (beaconOneRed) {
                    robot.MotorR.setPower(-.7 * vr);
                    robot.MotorL.setPower(-.3 * vl);
                }
                else {
                    robot.MotorR.setPower(-.3 * vr);
                    robot.MotorL.setPower(-.7 * vl);
                }
                /*if(lastPosR == robot.MotorR.getCurrentPosition()){
                    beaconOneCount++;
                }
                if(beaconOneCount > 10){
                    beaconOneDone = true;// stuck on wall
                }*/
                telemetry.addData("Status:", status);
                telemetry.addData("currentPos - startPosR + 400", robot.MotorR.getCurrentPosition() - startPosR + 330);
                telemetry.addData("beaconOneCount", beaconOneCount);
                telemetry.addData("beaconOneDone", beaconOneDone);
                telemetry.update();
                lastPosR = robot.MotorR.getCurrentPosition();
            }


        status = "forward off beacon 1";
        telemetry.update();
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 1550) {
            robot.MotorR.setPower(.6 * vr);
            robot.MotorL.setPower(.6 * vl);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorR to go", robot.MotorR.getCurrentPosition() - startPosR - 1550);
            telemetry.update();
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "turn to beacon 2";
        telemetry.update();
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 1100) {
            robot.MotorR.setPower(.5 * vr);
            robot.MotorL.setPower(-.5 * vl);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorR to go", robot.MotorR.getCurrentPosition() - startPosR + 1100);
            telemetry.update();
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "drive until white line";
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && (robot.MotorR.getCurrentPosition() > startPosR - 3700) && !longDriveDone) {//stop if hit line or go certain distance
            robot.MotorR.setPower(-.55 * vr);
            robot.MotorL.setPower(-.55 * vl);
            if(robot.colsensor.blue() > 8){
                longDriveDone = true;//hit white line
            }
            robot.pressservo.setPosition(0);
            telemetry.addData("Status:", status);
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "backward past line";
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(-.3 * vr);
        robot.MotorL.setPower(-.3 * vl);
        while (opModeIsActive() && (robot.MotorR.getCurrentPosition() > startPosR - 550) && !forwardTwoDone)  {//stop if go certain distance or stuck on wall
            if(lastPosR == robot.MotorR.getCurrentPosition()){
                forwardTwoCount++;
            }
            if(forwardTwoCount > 5){
                forwardTwoDone = true;//stuck on wall
            }
            telemetry.addData("Status:", status);
            telemetry.addData("MotorR units to go", robot.MotorR.getCurrentPosition() - startPosR + 550);
            telemetry.addData("MotorR current", robot.MotorR.getCurrentPosition());
            telemetry.addData("forwardTwoCount", forwardTwoCount);
            telemetry.addData("forwardTwoDone", forwardTwoDone);
            telemetry.update();
            lastPosR = robot.MotorR.getCurrentPosition();
        }

        status = "turn until white line";
        robot.MotorR.setPower(-.4 * vr);
        robot.MotorL.setPower(.4 * vl);
        while (opModeIsActive() && robot.colsensor.blue() < 8) {
            if(lastPosR == robot.MotorR.getCurrentPosition()){
                turnTwoCount++;
            }
            if(turnTwoCount > 5){
                turnTwoDone = true;//stuck on wall
            }
            if(turnTwoDone){//forward
                robot.MotorR.setPower(.4 * vr);
                robot.MotorL.setPower(.4 * vl);
                turnTwoCount = 0;
                turnTwoDone = false;
            }
            telemetry.addData("Status:", status);
            telemetry.addData("turnTwoCount", turnTwoCount);
            telemetry.addData("turnTwoDone", turnTwoDone);
            telemetry.update();
            lastPosR = robot.MotorR.getCurrentPosition();
        }

        status = "line follow";
        runtime.reset();
        while (opModeIsActive() && !robot.tsensor.isPressed()) {
            telemetry.addData("Status:", status);
            telemetry.addData("tsensor.isPressed()", robot.tsensor.isPressed());
            telemetry.update();
            if (robot.colsensor.blue() > 6) {//grey
                robot.MotorR.setPower(.25 * vr);
                robot.MotorL.setPower(-.5 * vl);
            } else if (robot.colsensor.blue() < 6) {//white
                robot.MotorR.setPower(-.5 * vr);
                robot.MotorL.setPower(.25 * vl);
            }
        }

        status = "sense color second";
        telemetry.update();
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .1) {
            telemetry.addData("Status:", status);
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            if (robot.fruitysensor.blue() > robot.fruitysensor.red()) {
                beaconTwoRed = false;
            }
            else {
                beaconTwoRed = true;
            }
        }

        status = "forward off beacon 2";
        telemetry.update();
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(.4 * vr);
        robot.MotorL.setPower(.4 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 150) {
            telemetry.addData("Status:", status);
            telemetry.addData("MotorR to go", robot.MotorR.getCurrentPosition() - startPosR - 150);
            telemetry.update();
        }

        status = "turn paddles second";
        telemetry.update();
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1) {
            telemetry.addData("Status:", status);
            telemetry.update();
            if (beaconTwoRed) {
                robot.pressservo.setPosition(.93);
            }
            else {
                robot.pressservo.setPosition(.36);
            }
        }

        status = "backward and press buttons, beacon 2";
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 300) {
            if (beaconTwoRed) {
                robot.MotorR.setPower(-.5 * vr);
                robot.MotorL.setPower(-.3 * vl);
            }
            else {
                robot.MotorR.setPower(-.3 * vr);
                robot.MotorL.setPower(-.5 * vl);
            }
            telemetry.addData("Status:", status);
            telemetry.addData("currentPos - startPosR + 400", robot.MotorR.getCurrentPosition() - startPosR + 400);
            telemetry.update();
        }


        status = "forward off beacon 2";
        telemetry.update();
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 1000) {
            robot.MotorR.setPower(.4 * vr);
            robot.MotorL.setPower(.4 * vl);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorR to go", robot.MotorR.getCurrentPosition() - startPosR - 1000);
            telemetry.update();
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "turn to cap ball";
        telemetry.update();
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 650) {
            robot.MotorR.setPower(-.4 * vr);
            robot.MotorL.setPower(.4 * vl);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorR to go", robot.MotorR.getCurrentPosition() - startPosR + 650);
            telemetry.update();
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);


        status = "forward to cap ball";
        telemetry.update();
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 4000) {
            robot.MotorR.setPower(.4 * vr);
            robot.MotorL.setPower(.4 * vl);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorR to go", robot.MotorR.getCurrentPosition() - startPosR - 4000);
            telemetry.update();
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);
        }


    }



