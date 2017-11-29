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
    private DcMotor motorFR, motorBR, motorFL, motorBL;
    private Servo squeeze; // also, this goes in port one of the servo controller
    private DcMotor motor_lift;
    /* ---------------------------------------- */

    @Override
    public void init()
    {   //Assign values to hardware components here (match them to phone configuration)
        // Motor and motor controller hardware declaration
        /* ---------------------------------------- */
        motorControllerDrive = hardwareMap.dcMotorController.get("MC_D");

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        /* ---------------------------------------- */
        // Encoder stuff - Run Without Encoders is depreciated
        /* ---------------------------------------- */
        //motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        //motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        // Flipped the motors (11/10/17)
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
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
        
        /*rightPower = gearRatio * gamepad1.right_stick_y;
        leftPower = gearRatio * gamepad1.left_stick_y;*/

        /* If the value of the power is lower than the threshold, the robot will set its power to
         the threshold */
        double threshold = 0.1;
        leftPower = (leftPower > 0 && leftPower < threshold) ? threshold : leftPower;
        leftPower = (leftPower < 0 && leftPower > -threshold) ? -threshold : leftPower;
        rightPower = (rightPower > 0 && rightPower < threshold) ? threshold : rightPower;
        rightPower = (rightPower < 0 && rightPower > -threshold) ? -threshold : rightPower;

        // Clips the power
        //leftPower = Range.clip(leftPower, -1, 1);        //gamepad controllers have a value of 1 when you push it to its maximum foward
        //rightPower = Range.clip(rightPower, -1, 1);      //limiting the range of each power, min first then max

        // For flipping the robot's front
        if(flip_front)
        {
            leftPower *= -1;
            rightPower *= -1;
        } 

        // Set the robot's power
       /* motorFR.setPower(rightPower);
        motorBR.setPower(-rightPower); // back motors need to be reversed because of the gears
        motorFL.setPower(leftPower);
        motorBL.setPower(-leftPower); */ // back motors need to be reversed because of the gears

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
        while(gamepad2.right_bumper)
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
        }


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


        // Lightning's teleop from last year
        x = gamepad1.left_stick_x ;
       
        double frontRight, frontLeft, backRight, backLeft;

        y = -gamepad1.left_stick_y;//only y is inversed

        z = gamepad1.right_stick_x; //handles the robot turning

        x = Range.clip(x, -1, 1);
        y = Range.clip(y, -1, 1);
        z = Range.clip(z, -1, 1);

        if(gamepad1.dpad_right && speed < 1)
        {
            change_speed = 1;
        }
        if(gamepad1.dpad_left && speed > 0)
        {
            change_speed = 2;
        }

        if(change_speed != prev)
        {
            if(change_speed == 1)
            {
                speed += 0.1;
            }

            if(change_speed == 2)
            {
                speed -= 0.1;
            }
        }

        prev = change_speed;
        change_speed = 0;


        if(speed > 1)
        {
            speed = 1;
        }

        if(speed < 0)
        {
            speed = 0;
        }
        Range.clip(speed, 0, 1);


        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("z", z);
        telemetry.update();

        telemetry.addData("Speed", speed);
        telemetry.update();

        x *= speed;
        y *= speed;
        z *= speed;

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("z", z);
        telemetry.update();

//        x = x * x * x;
//        y = y * y * y;
//        z = z * z * z;

        power = Math.sqrt(x * x + y * y);

        if(gamepad1.dpad_down)
        {
            if(swap_front_back)
            {
                swap_front_back = false;
            }
            else
            {
                swap_front_back = true;
            }
        }

        int zone = -1;// 0 is right, 1 is up-right, 2 is up, 3 is up-left,
        // 4 is left, 5 is down-left, 6 is down, 7 is down-right

        if(y == 0) {
            // Right
            if (x > 0) zone = 0;
            // Left
            if (x < 0) zone = 4;
            // Nothing
            if (x == 0) zone = -1;
        }
        if(y > 0){
            // Forward
            if (x == 0) zone = 2;
            // Up-Right
            if (x > 0) {
                slope = Math.sqrt(2);
                // If up
                if (x * slope < y) zone = 2;
                else
                {
                    slope = 1 / slope;
                    // If right
                    if (x * slope > y) zone = 0;
                    else zone = 1;
                }
            }
            // up-left
            if (x < 0) {
                slope = -Math.sqrt(2);
                // if up
                if (x * slope < y) zone = 2;
                else
                {
                    slope = 1 / slope;
                    // if left
                    if (x * slope > y) zone = 4;
                    else zone = 3;
                }
            }
        }
        if(y < 0){
            // Backward
            if (x == 0) zone = 6;
            // Down-Right
            if (x > 0) {
                slope = -Math.sqrt(2);
                // If down
                if (x * slope > y) zone = 6;
                else
                {
                    slope = 1 / slope;
                    // If right
                    if (x * slope < y) zone = 0;
                    else zone = 7;
                }
            }
            // down-left
            if (x < 0) {
                slope = Math.sqrt(2);
                // if up
                if (x * slope > y) zone = 6;
                else
                {
                    slope = 1 / slope;
                    // if left
                    if (x * slope < y) zone = 4;
                    else zone = 5;
                }
            }
        }
        if(zone == 0)// right
        {
            frontLeft = backRight = power;
            backLeft = frontRight = -power;
        }
        if(zone == 4)// left
        {
            LF = RB = -power;
            LB = RF = power;
        }
        if(zone == 2)// up
        {
            LF = RB = power;
            LB = RF = power;
        }
        if(zone == 6)// down
        {
            LF = RB = -power;
            LB = RF = -power;
        }


        if(zone == 1)// up-right
        {
            LF = RB = power;
            LB = RF = 0;
        }
        if(zone == 3)// up-left
        {
            LF = RB = 0;
            LB = RF = power;
        }
        if(zone == 5)// down-left
        {
            LF = RB = -power;
            LB = RF = 0;
        }
        if(zone == 7)// down-right
        {
            LF = RB = 0;
            LB = RF = -power;
        }

        z *= -1;

        if(swap_front_back)
        {
            LF *= -1;
            RF *= -1;
            LB *= -1;
            RB *= -1;

            z *= -1;
        }

        LF = ((LF) + z) * 0.707;
        RF = ((RF) - z) * 0.707;
        LB = ((LB) + z) * 0.707;
        RB = ((RB) - z) * 0.707;

        LF = Range.clip(LF, -1, 1);
        RF = Range.clip(RF, -1, 1);
        LB = Range.clip(LB, -1, 1);
        RB = Range.clip(RB, -1, 1);



        //set power here
        motor_FR.setPower(RF);
        motor_FL.setPower(LF);
        motor_BR.setPower(RB);
        motor_BL.setPower(LB);
        
    }
}
