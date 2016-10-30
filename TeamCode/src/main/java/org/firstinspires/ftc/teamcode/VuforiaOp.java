package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name = "VuforiaOp", group = "Tele")

public class VuforiaOp extends OpMode{

    HardwarePushbotTDR robot   = new HardwarePushbotTDR();
    public DcMotor motorR;
    public DcMotor motorL;
    private ElapsedTime runtime = new ElapsedTime();
    int count = 0;

    private VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);


    private VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);


    private VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");




    public VuforiaOp(){

    }

    @Override
    public void init() {

        robot.init(hardwareMap);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AQCaDbr/////AAAAGbB9PlSG00NXpbiQDePjukiBHcOPuC5f4vbHarDMkZ8MRj8pXwuCkfPuFTJQ69948uKuCBQzqPaxqO9HnowiGP6cZ+1qXrxfqyaty8NDE67dTSnjqX2WgfCrkwCpT1RJT76v3e5br0mC/tR5F/b8FWgxs4XeB7kUv23wFiibZDWIFfjJQQKrtnzRZs6jIcu8E24T0pUl0e6PapM86x6gPiCoVqq2IuPMNc1aA1KQVWC05TjfnLpEmnUHn6Gx8Ue+VFMRWXRJgITNa7FQfQ9fuqyeS9sAz5uGE3m/GS0DECfB6lCuqXp4HGVaLFPg/dFwIki4O8A04kDFtVxoYEIk+XNlgs2nxd/RKT6q6f7udlVW";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");
        beacons.activate();
    }

    @Override
    public void loop(){


            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to run");    //
            telemetry.update();
            count++;

            telemetry.addData("Loop", "Running");
            double l;
            double r;


            l = -gamepad1.left_stick_y;
            r = -gamepad1.right_stick_y;

            telemetry.addData("l value:", l);
            telemetry.addData("r value:", r);

            if (l < -0.05 || l > 0.05) {
                robot.MotorL.setPower(l);
            } else {
                robot.MotorL.setPower(0);
            }

            if (r < -0.05 || r > 0.05) {
                robot.MotorR.setPower(r);
            } else {
                robot.MotorR.setPower(0);
            }


            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if (pose != null) {

                    VectorF translation = pose.getTranslation();

                    double sign1x = translation.get(1);
                    double sign1z = translation.get(2);
                    double sign1y = translation.get(0);

                    telemetry.addData(beac.getName() + "-Translation", translation);
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);

                    telemetry.addData(beac.getName() + "X: ", sign1x);
                    telemetry.addData(beac.getName() + "Y: ", sign1y);
                    telemetry.addData(beac.getName() + "Z: ", sign1z);

                }
            }

            telemetry.update();


        }



    @Override
    public void stop() {
    }


}