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

    }

    public void loop()
    {
        /** Declaring variables that are used **/
        /* -------------------------------------------------------------------------------------- */
        // Variables for controlling movement
        double rightPower, leftPower;
        double gearRatio;

        // Other variables
        boolean gear_ratio_is_07 = true;
        boolean flip_front = false;
        /* -------------------------------------------------------------------------------------- */


        /** Changing driving mode section
         *   When the left_bumper is pressed, the robot will switch
         *   When the right_bumper is pressed, the robot will decrease its software gear ratio and
         *   consequently move slower. This allows for more precise movements.
         *   **/
        /* -------------------------------------------------------------------------------------- */
        // If right_bumper toggles gear ratio, the default gearRatio is 0.7. Otherwise, the gearRatio is 0.2
        if (gamepad1.right_bumper)
        {
            gear_ratio_is_07 = !(gear_ratio_is_07);
        }
        gearRatio = gear_ratio_is_07 ? 0.7 : 0.2;

        // toggles front
        if (gamepad1.left_bumper)
        {
            flip_front = !(flip_front);
        }
        /* -------------------------------------------------------------------------------------- */


        /**
         * Gets position of the joysticks on controller1
         * and sets the power of the motors as the position multiplied by a constant
         * to influence the potency of the motors.
         */
        /* -------------------------------------------------------------------------------------- */
        // Allows gear ratio to affect the motor power
        rightPower = gearRatio * gamepad1.right_stick_y;
        leftPower = gearRatio * gamepad1.left_stick_y;

        // Since motor power is between -1 and 1 inclusive, range should be clipped if too extreme
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        // If flip_front is true, then flip motor direction
        if (flip_front)
        {
            leftPower *= -1;
            rightPower *= -1;
        }

        // Set motor power, move accordingly
        motorR.setPower(rightPower);
        motorL.setPower(leftPower);
        /* -------------------------------------------------------------------------------------- */


        /** Telemetry for debugging and convenience **/
        /* -------------------------------------------------------------------------------------- */
        telemetry.addData("Gear Ratio ", gearRatio);
        telemetry.addData("Right Power ", rightPower);
        telemetry.addData("Left Power ", leftPower);
        telemetry.addData("Front Flipped ", flip_front);

        telemetry.update();
        /* -------------------------------------------------------------------------------------- */
        
    }
}

