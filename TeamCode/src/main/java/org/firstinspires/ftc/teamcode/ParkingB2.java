package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Justin Zhu
 * NOTE:
 * This is for B2 (Bottom Blue)
 */
@Autonomous(name="Parking B2", group="Autonomous")
public class ParkingB2 extends AutonomousMethodMaster{
    public void runOpMode() {

        initElectronics(0);

        waitForStart();

//        encoderMove(0.2,  28, 28);  // move forward 28 in
//        encoderRotateDegrees(0, 0.2, 90); //rotate cw 90 degrees
//        encoderMove(0.2, 8, 8); //move forward 8 in
//        stopMotion(); //should be parked by now

        parkB2();
    }
}
