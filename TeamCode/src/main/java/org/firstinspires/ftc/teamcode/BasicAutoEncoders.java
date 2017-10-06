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
 * Created by joshuakrinsky on 9/29/17.
 */
@Autonomous(name="Encoder", group="Autonomous")
@Disabled
public class BasicAutoEncoders extends AutonomousMethodMaster{
    private DcMotor motorL;                       // Left Side Motor
    private DcMotor motorR;                       // Right Side Motor

    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.875 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        motorL = hardwareMap.dcMotor.get("motorL");        //P0 is actually the right
        motorR = hardwareMap.dcMotor.get("motorR");        //P1 is actually the left

        encoderMode(3);
        encoderMode(1);

        waitForStart();

        encoderMove(10,  10, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderMove(12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderMove( -10, -10, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        sleep(1000);


    }
}
