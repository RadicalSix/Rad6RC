package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "auto1TestsVuforia", group = "Auto")

public class Auto1TestsVuforia extends LinearOpMode {

    HardwarePushbotSimple robot = new HardwarePushbotSimple();
    VuforiaOp camera = new VuforiaOp();
    public DcMotor motorR;
    public DcMotor motorL;
    private ElapsedTime runtime = new ElapsedTime();
    public ColorSensor colsensor;
    Boolean beaconOneRed;

    double vl = 1;
    double vr = 1;
    int step = 0;


    /*
        public AutonomousMDR(){

        }
    */
    @Override
    public void runOpMode() throws InterruptedException {


        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ATVwosb/////AAAAGYlO5qoc6kZagqZX6jvBKGgVjiVqbwuCKCZeIQTBkfNwsuJY/+oa3DHJbR/aFFfPF2A/bsi9cY36hUzYuOhFVBmWjYzVbQEh3YPoVATeaQEr/P6hNDA2AbW1Xbq0+hxqiYKpA1vNu22pVPOMW7MDmDst4HiuDLEXATZC3boSoLU6d9up0qPxZbZ+3fjXMnMTr6QkXIle3O7dfg/FVM09i/CIsq/Harcgg6lCoOYnrw70TEmPXOAxYdMh6Dh2KxZ8uAfHLur0U2adA0mWUKS7+z8Axq6jlH5oY8LOXp0FqX6A820mkqeDZz5DCkupkLOuTw/taIqz4vf2ewHRB8xGt7hEu34ZOr1TWOpT0bVnLLhB";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        double x = 0;
        double y = 0;
        double z = 0;
        double degreesToTurn = 0;



        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");
        beacons.activate();

        robot.init(hardwareMap);

        double startPosR = robot.MotorR.getCurrentPosition();
        robot.pressservo.setPosition(0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();





        //Oct 16- start robot with phone on/off button almost touching wall, black zip tie on beam above right motor in line with left side of floor mat ridge


        //back into ball
        step = 1;
        robot.MotorR.setPower(-.6*vr);
        robot.MotorL.setPower(-.6*vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 2000) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPos - startPosR + 3400", robot.MotorR.getCurrentPosition() - startPosR + 2000);
            telemetry.update();
            //idle();
        }


        //wait
        step = 2;
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
        step = 3;
        robot.MotorL.setPower(.7*vl);
        robot.MotorR.setPower(-.7*vr);
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 1200) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("currentPos - startPosR + 1200 - ", robot.MotorR.getCurrentPosition() - startPosR + 1200);
            telemetry.update();
            //idle();
        }

        //wait
        step = 4;
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
        step = 5;
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
                    robot.MotorL.setPower(.5 * vl);
                    robot.MotorR.setPower(.2 * vr);
                    telemetry.addData("right", x);
                } else if (x < 0) {
                    robot.MotorL.setPower(.2 * vl);
                    robot.MotorR.setPower(.5 * vr);
                    telemetry.addData("left", x);
                } /*else {//didnt turn enough
                    robot.MotorL.setPower(.7 * vl);
                    robot.MotorR.setPower(-.7 * vr);
                }*/
                if (z > -500) {
                    done = true;
                }
            }

            telemetry.addData("found:", found);
            telemetry.addData("done:", done);
            telemetry.update();
        }

        //wait
        step = 6;
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
        step = 7;
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
                        robot.MotorL.setPower(-.5 * vl);
                        robot.MotorR.setPower(.5 * vr);
                        telemetry.addData("right", x);
                    } else if (degreesToTurn < 179 && degreesToTurn > 90) {
                        robot.MotorL.setPower(.5 * vl);
                        robot.MotorR.setPower(-.5 * vr);
                        telemetry.addData("left", x);
                    }
                }


            if ((degreesToTurn < -175 && degreesToTurn > -181) || (degreesToTurn > 175 && degreesToTurn < 181)){
                done2 = true;
            }
            telemetry.update();
        }

        //wait
        step = 8;
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
        step = 7;
        startPosR = robot.MotorR.getCurrentPosition();
        robot.MotorR.setPower(-.5 * vr);
        robot.MotorL.setPower(.5 * vl);
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() > startPosR - 1200) {
            telemetry.addData("Step:", step);
            telemetry.addData("currentPosR", robot.MotorR.getCurrentPosition());
            telemetry.addData("currentPos - startPosR + 1200", robot.MotorR.getCurrentPosition() - startPosR + 1200);
            telemetry.update();
            idle();
        }

        //wait
        step = 8;
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
        step = 9;
        robot.MotorR.setPower(.5 * vr);
        robot.MotorL.setPower(.5 * vl);
        while (opModeIsActive() && robot.colsensor.blue() < 6) {//changed from 6 to 10 10/16
            telemetry.addData("Step:", step);
            telemetry.addData("sensorColor:", robot.colsensor.blue());
            telemetry.update();
            idle();
        }
        //wait
        step = 6;
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
        step = 7;
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
        step = 10;
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

        step = 10;
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
        step = 11;
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
        step = 12;
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        startPosR = robot.MotorR.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            telemetry.addData("Step:", step);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        step =13;
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

        step = 14;
            while(opModeIsActive() && robot.tsensor.isPressed()){
                robot.MotorL.setPower(.5 * vl);
                robot.MotorR.setPower(.5 * vr);
            }

        //wait
        step = 12;
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
        step = 15;
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
        step = 14;
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
        step = 15;
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
        step = 14;
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
        step = 15;
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
        step = 16;
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
        step = 17;
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
        step = 18;
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
        step = 19;
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
        step = 20;
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
        step = 21;
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
        step = 22;
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
        step = 23;
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
        step = 24;
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
        step = 25;
        if (robot.fruitysensor.blue() > robot.fruitysensor.red()) {
            robot.pressservo.setPosition(.9);

        }
        else {
            robot.pressservo.setPosition(.0);
        }

        //wait
        step = 26;
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
        step = 27;
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
        step = 28;
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
    }
}
