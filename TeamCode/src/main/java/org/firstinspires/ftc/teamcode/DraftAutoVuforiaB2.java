package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;

/**
 * Created by justin on 12/1/17.
 */
//@Autonomous(name="VuforiaB2", group="Autonomous")
public class DraftAutoVuforiaB2 extends AutonomousMethodMaster{

    public void runOpMode()
    {
        initElectronics(0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // the licence key that vuforia needs to work
        parameters.vuforiaLicenseKey = "AQRacK7/////AAAAGea1bsBsYEJvq6S3KuXK4PYTz4IZmGA7SV88bdM7l26beSEWkZTUb8H352Bo/ZMC6krwmfEuXiK7d7qdFkeBt8BaD0TZAYBMwHoBkb7IBgMuDF4fnx2KiQPOvwBdsIYSIFjiJgGlSj8pKZI+M5qiLb3DG3Ty884EmsqWQY0gjd6RNhtSR+6oiXazLhezm9msyHWZtX5hQFd9XoG5npm4HoGaZNdB3g5YCAQNHipjTm3Vkf71rG/Fffif8UTCI1frmKYtb4RvqiixDSPrD6OG6YmbsPOYUt2RZ6sSTreMzVL76CNfBTzmpo2V0E6KKP2y9N19hAum3GZu3G/1GEB5D+ckL/CXk4JM66sJw3PGucCs";

        // indicates camera direction
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // loads data for vumarks
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        waitForStart();

        // blue2
        //encoderStrafeRight(1, 24);

        int move_inches = 0;
        int timesChecked = 0;

        // identify which vumark
        while (vuMark == null && timesChecked <= 3){
            //motorL.setPower(0.25);
            //motorR.setPower(0.25);
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            sleep(250);
            timesChecked ++;
            if(timesChecked %2 ==1) {
                telemetry.addData("VuMark", "is not visible; re-aligning");
                telemetry.update();
                encoderStrafeRight(1, 1);
            }
            else
            {
                telemetry.addData("VuMark", "is not visible; re-aligning");
                telemetry.update();
                encoderStrafeRight(1, -1);
            }
        }
        if(vuMark == RelicRecoveryVuMark.LEFT){
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();

            move_inches = 4;
        }
        else if(vuMark == RelicRecoveryVuMark.CENTER){
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();

            move_inches = 12;
        }
        else if(vuMark == RelicRecoveryVuMark.RIGHT){
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();

            move_inches = 20;
        }
        else{
                telemetry.addData("VuMark", "is not visible; Moving to center");
                telemetry.update();
                move_inches = 12;
        }

        /* We further illustrate how to decompose the pose into useful rotational and
         * translational components */
        double tX = 0, tY = 0, tZ = 0;
        while (vuMark != RelicRecoveryVuMark.UNKNOWN && (tY < 20 * inchToMm)) // 20 as in 20 inches
        {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));
            if(pose != null) {
                VectorF trans = pose.getTranslation();

                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                tX = trans.get(0);
                tY = trans.get(1);
                tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;

                telemetry.addData("X translation", tX);
                telemetry.addData("Y translation", tY);
                telemetry.addData("Z translation", tZ);

                telemetry.addData("X rotation", rX);
                telemetry.addData("Y rotation", rY);
                telemetry.addData("Z rotation", rZ);

                telemetry.update();

                encoderMove(1, -2, -2); // just move...
            }
        }

        encoderRotateDegrees(0, 1, 90); // rotate into direction

        encoderStrafeRight(1, move_inches); // move direction based on VuMark

        encoderMove(1, 12, 12); // move forward to position
        encoderMove(.5, -32 + move_inches, -32 + move_inches); // move direction based on VuMark

        encoderRotateDegrees(0,0.5,90);
        
        encoderMove(0.5, 4,4);
        
        encoderRotateDegrees(0,0.5,90);
        
        encoderMove(0.5, 2,2);


        dumpGlyph(); // dump the glyph
    }

}
