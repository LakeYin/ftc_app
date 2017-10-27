/*
 * V 2.0 - 
 * Add fuctionality of changing the power given to motors based on state of the right bumper
 * pressed makes gear ratio .7 while non pressed makes gear ratio .3
 * 
 * V 1.0 - Loops through and sets power to motors based on joystick position
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by user on 9/22/2017.
 */

@TeleOp(name="BasicTeleop", group = "Basic")
public class BasicTeleop extends OpMode
{
    // Initialize the components of the robot
    /* ---------------------------------------- */
    private DcMotorController motorControllerDrive;
    private DcMotor motorR, motorL;
    private Servo squeeze; // also, this goes in port one of the servo controller
    /* ---------------------------------------- */


    @Override
    public void init()
    {   //Assign values to hardware components here (match them to phone configuration)
        // Motor and motor controller hardware declaration
        /* ---------------------------------------- */
        motorControllerDrive = hardwareMap.dcMotorController.get("MC_D");

        motorR = hardwareMap.dcMotor.get("motorR");
        motorL = hardwareMap.dcMotor.get("motorL");
        /* ---------------------------------------- */

        // Encoder stuff - Run Without Encoders is depreciated
        /* ---------------------------------------- */
        //motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        //motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        motorR.setDirection(DcMotor.Direction.REVERSE);
        //motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        /* ---------------------------------------- */

        squeeze = hardwareMap.servo.get("squeeze");

    }

    double squeezePosition = 1;

    public void loop()
    {   /*
         * Gets position of the joysticks on controller1
         * and sets the power of the motors as the position multiplied by a constant
         * to influence the potency of the motors.
         */
        boolean gear_ratio_is_07 = true;
        boolean flip_front = false;
        double rightPower, leftPower;
        double gearRatio;

        if(gamepad1.right_bumper)
        {
            gear_ratio_is_07 = !(gear_ratio_is_07);
        }
        gearRatio = gear_ratio_is_07 ? 0.7 : 0.2;
        // If right_bumper toggles gear ratio, the default gearRatio is 0.7. Otherwise, the gearRatio is 0.2

        if(gamepad1.left_bumper) // toggles front
        {
            flip_front = !(flip_front);
        }
        
        rightPower = gearRatio * gamepad1.right_stick_y;
        leftPower = gearRatio * gamepad1.left_stick_y;
        
        leftPower = Range.clip(leftPower, -1, 1);        //gamepad controllers have a value of 1 when you push it to its maximum foward
        rightPower = Range.clip(rightPower, -1, 1);      //limiting the range of each power, min first then max

        if(flip_front)
        {
            leftPower *= -1;
            rightPower *= -1;
        }
        
        motorR.setPower(rightPower);
        motorL.setPower(leftPower);

        /*if(gamepad2.right_bumper) // 83.3 is the number of degrees from 0 to snuggly fit the glyph
        {
            squeezePosition += 0.0001;
        }
        if(gamepad2.left_bumper)
        {
            squeezePosition -= 0.0001;
        }*/
        
        //while loops should make the clamp gradually open and close
        final double DEGREE_CHANGER =.0001;
        while(gamepad2.right_bumper)
        {
            squeezePosition += DEGREE_CHANGER;
            sleep(10);
        }
        while(gamepad2.left_bumper)
        {
            squeezePosition += (DEGREE_CHANGER*-1); 
            sleep(10);
        } 
        
        
        

        //squeezePosition = -gamepad2.right_trigger + 1; // defaults to open
        squeezePosition = gamepad2.right_trigger; // defaults to closed

        squeezePosition = Range.clip(squeezePosition, 82/180, 1);
        squeeze.setPosition(squeezePosition);
          
        telemetry.addData("Gear Ratio ", gearRatio);
        telemetry.addData("Right Power ", rightPower);
        telemetry.addData("Left Power ", leftPower);
        telemetry.addData("Squeeze Position * 180", squeezePosition * 180);
        
        
    }
}

