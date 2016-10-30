package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
 * Created by 12sho_000 on 10/23/2016.
 */

@Autonomous(name = "VuforiaOp1", group = "Auto")
public class VuforiaOp1 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ATVwosb/////AAAAGYlO5qoc6kZagqZX6jvBKGgVjiVqbwuCKCZeIQTBkfNwsuJY/+oa3DHJbR/aFFfPF2A/bsi9cY36hUzYuOhFVBmWjYzVbQEh3YPoVATeaQEr/P6hNDA2AbW1Xbq0+hxqiYKpA1vNu22pVPOMW7MDmDst4HiuDLEXATZC3boSoLU6d9up0qPxZbZ+3fjXMnMTr6QkXIle3O7dfg/FVM09i/CIsq/Harcgg6lCoOYnrw70TEmPXOAxYdMh6Dh2KxZ8uAfHLur0U2adA0mWUKS7+z8Axq6jlH5oY8LOXp0FqX6A820mkqeDZz5DCkupkLOuTw/taIqz4vf2ewHRB8xGt7hEu34ZOr1TWOpT0bVnLLhB";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        waitForStart();

        beacons.activate();

        while(opModeIsActive()){
            for(VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if(pose != null){

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

    }
}

