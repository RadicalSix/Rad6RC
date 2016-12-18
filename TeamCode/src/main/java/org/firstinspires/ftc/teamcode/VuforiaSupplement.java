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

//@Autonomous(name = "auto3TestsVuforiaRed", group = "Auto")

public class VuforiaSupplement extends LinearOpMode {

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
        robot.liftservo.setPosition(.03);
        robot.shotFeeder.setPosition(.9);
        robot.pressservoR.setPosition(.0);
        robot.conveyorservo.setPosition(0);//in

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //drives towards the center of the picture until a measured distance away
        status = "drive to picture until z = -200";
        telemetry.update();
        boolean done = false;
        boolean found = false;
        while (opModeIsActive() && !done) {
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
            if (z < 0) {
                found = true;//does not compare values until phone has found picture>>> get value for z
            }
            if (found) {
                if (x > 0) {//center of the picture is to the right
                    robot.MotorL.setPower(1 * vl);
                    robot.MotorR.setPower(.5 * vr);
                    telemetry.addData("right", x);
                } else if (x < 0) {//center of the picture is to the left
                    robot.MotorL.setPower(.5 * vl);
                    robot.MotorR.setPower(1 * vr);
                    telemetry.addData("left", x);
                } else {//didnt turn enough
                    robot.MotorL.setPower(.7 * vl);
                    robot.MotorR.setPower(-.7 * vr);
                }
                if (z > -200) {
                    done = true;//loop will not run again because have reached -200
                }
            }

            //now that laterally centered with picture, turn so that facing picture square on
            status = "turn until square with beacon";
            telemetry.update();
            boolean done2 = false;
            found = false;
            while (opModeIsActive() && !done2) {
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
                if (z < 0) {
                    found = true;//does not compare values until phone has found picture>>> get value for z
                }
                if (found) {
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
                if ((degreesToTurn < -170 && degreesToTurn > -181) || (degreesToTurn > 170 && degreesToTurn < 181)) {
                    done2 = true;//now that roughly square with picture, loop will stop
                }
                telemetry.update();
            }
        }
    }
}

