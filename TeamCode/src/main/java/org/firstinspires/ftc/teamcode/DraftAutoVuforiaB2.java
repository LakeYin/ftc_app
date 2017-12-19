package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by justin on 12/1/17.
 */
@Autonomous(name="VuforiaB2", group="Autonomous")
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

        // red1
        encoderStrafeRight(1, 24);

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

        encoderRotateDegrees(0, 1, 90); // rotate into direction

        encoderStrafeRight(1, move_inches); // move direction based on VuMark

        encoderMove(1, 12, 12); // move forward to position

        dumpGlyph(); // dump the glyph
    }

}
