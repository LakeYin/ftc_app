package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;


import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by Justin Zhu
 * NOTE:
 * This is for R2 (Bottom Red)
 */
@Autonomous(name="Parking R2", group="Autonomous")
public class ParkingR2 extends AutonomousMethodMaster{
    public void runOpMode() {

        initElectronics(0);

        waitForStart();

//        encoderMove(0.2,  28, 28);  // move forward 28 in
//        encoderRotateDegrees(1, 0.2, 90); //rotate ccw 90 degrees
//        encoderMove(0.2, 8, 8); //move forward 8 in
//        stopMotion(); //should be parked by now

        parkR2();
        dumpGlyph();
        stopMotion();                                                   //Stops all motors - a failsafe for our failsafe.
    }
}
