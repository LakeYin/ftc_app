/*
 * V 2.0 - 
 * Add fuctionality of changing the power given to motors based on state of the right bumper
 * pressed makes gear ratio .7 while non pressed makes gear ratio .3
 * 
 * V 1.0 - Loops through and sets power to motors based on joystick position
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private DcMotor motor_lift;
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

        // Flipped the motors (11/10/17)
        motorR.setDirection(DcMotor.Direction.REVERSE);
        //motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        /* ---------------------------------------- */

        squeeze = hardwareMap.servo.get("squeeze");
        motor_lift = hardwareMap.dcMotor.get("lift");
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

        if(gamepad1.left_bumper) // toggles front, kinda unnecessary though
        {
            flip_front = !(flip_front);
        }
        
        rightPower = gearRatio * gamepad1.right_stick_y;
        leftPower = gearRatio * gamepad1.left_stick_y;

        /* If the value of the power is lower than the threshold, the robot will set its power to
         the threshold */
        double threshold = 0.1;
        leftPower = (leftPower > 0 && leftPower < threshold) ? threshold : leftPower;
        leftPower = (leftPower < 0 && leftPower > -threshold) ? -threshold : leftPower;
        rightPower = (rightPower > 0 && rightPower < threshold) ? threshold : rightPower;
        rightPower = (rightPower < 0 && rightPower > -threshold) ? -threshold : rightPower;

        // Clips the power
        leftPower = Range.clip(leftPower, -1, 1);        //gamepad controllers have a value of 1 when you push it to its maximum foward
        rightPower = Range.clip(rightPower, -1, 1);      //limiting the range of each power, min first then max

        // For flipping the robot's front
        if(flip_front)
        {
            leftPower *= -1;
            rightPower *= -1;
        } 

        // Set the robot's power
        motorR.setPower(rightPower);
        motorL.setPower(leftPower);

        /*final double DEGREE_CHANGER = 0.0001;

        if(gamepad2.right_bumper) // 83.3 is the number of degrees from 0 to snuggly fit the glyph
        {
            squeezePosition += DEGREE_CHANGER;
        }
        if(gamepad2.left_bumper)
        {
            squeezePosition -= DEGREE_CHANGER;
        }*/

        final double FIT_GLYPH = 82 / 180; // will (probably) always have glyph squeezed

        
        //while loops should make the clamp gradually open and close
        /*while(gamepad2.right_bumper)
        {
            squeezePosition += DEGREE_CHANGER;
            squeezePosition = Range.clip(squeezePosition, FIT_GLYPH, 1);
            squeeze.setPosition(squeezePosition);
            AutonomousMethodMaster.sleepNew(10);
        }
        while(gamepad2.left_bumper)
        {
            squeezePosition += (DEGREE_CHANGER*-1);
            squeezePosition = Range.clip(squeezePosition, FIT_GLYPH, 1);
            squeeze.setPosition(squeezePosition);
            AutonomousMethodMaster.sleepNew(10);
        }*/


        // dealing with the motor controlling the lift
        double lift_power;

        lift_power = gamepad2.right_stick_y;
        if(lift_power < 0)
        {
            lift_power *= lift_power;
            // If the lift_power is too small, set it to 0.1
            if (lift_power < 0.1 && lift_power != 0)
            {
                lift_power = 0.1;
            }
        }
        else
        {
            lift_power *= lift_power;
            // If the lift_power is too small, set it to 0.1
            if (lift_power < 0.1 && lift_power != 0)
            {
                lift_power = 0.1;
            }
            lift_power *= -1;
        }

        motor_lift.setPower(lift_power);

        //squeezePosition = -gamepad2.right_trigger + 1;   // defaults to open
        squeezePosition = gamepad2.right_trigger;          // defaults to closed
        boolean locked = false;                            // whether or not the right bumper has been pressed. Defaults to false.

        if(gamepad2.right_bumper && gamepad2.right_trigger > 0){ //ensures that we don't accidentally lock it open, which could be confusing to the drivers
            locked = !(locked);                            // toggles the squeeze boolean
        }

        if(!locked) {                                      // doesn't update position (locks the position) if the bumper hasn't been pressed
            squeezePosition = Range.clip(squeezePosition, FIT_GLYPH, 1);
            squeeze.setPosition(squeezePosition);
        }

          
        telemetry.addData("Gear Ratio ", gearRatio);
        telemetry.addData("Right Power ", rightPower);
        telemetry.addData("Left Power ", leftPower);
        telemetry.addData("Squeeze Position * 180", squeezePosition * 180);
        telemetry.addData("Lift Power ", lift_power);
        
    }
}

