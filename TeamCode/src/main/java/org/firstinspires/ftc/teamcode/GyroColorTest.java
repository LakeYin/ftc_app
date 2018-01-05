package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/**
 * Created by Lake Yin on 12/20/2017.
 */


@Autonomous(name="GyroColorTest", group = "Test")
public class GyroColorTest extends AutonomousMethodMaster {

    public void runOpMode(){

        initElectronics(0); // initialize and set up everything
        //setUpGyroScopeHT();
        //setUpColourSensor();

        while(opModeIsActive())
        {
            /*
            telemetry.addData("GyroDegrees X", "02x", NxtGyroSensor.rawX()); // all of the possible degrees
            telemetry.addData("GyroDegrees Y", "02x", NxtGyroSensor.rawY());
            telemetry.addData("GyroDegrees Z", "02x", NxtGyroSensor.rawZ());
            */

            telemetry.addData("Red", "02x", colorSensor.red()); // values of all of the colors
            telemetry.addData("Blue", "02x", colorSensor.blue());
            telemetry.addData("Green", "02x", colorSensor.green());

            telemetry.update();
        }
    }

}
