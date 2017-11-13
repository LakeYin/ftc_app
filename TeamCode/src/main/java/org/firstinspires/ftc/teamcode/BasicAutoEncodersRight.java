package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Justin Zhu
 */
@Autonomous(name="Basic Encoders Turn Right", group="Autonomous")
public class BasicAutoEncodersRight extends AutonomousMethodMaster{
    public void runOpMode() {

        initElectronics(0);

        waitForStart();

        encoderMove(0.2,  28, 28);  // move forward 12 in
        encoderRotateDegrees(0, 0.2, 90); //rotate cw 90 degrees
        encoderMove(0.2, 8, 8); //move forward 6 in
        stopMotion(); //should be parked by now
    }
}
