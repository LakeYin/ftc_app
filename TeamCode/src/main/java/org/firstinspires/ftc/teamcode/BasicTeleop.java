/*
 * V 2.0 - 
 * Add functionality of changing the power given to motors based on state of the right bumper
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
    /** Initialize the components of the robot **/
    /** ------------------------------------------------------------------------------------ **/
    //private DcMotorController motorControllerDrive;
    private DcMotor motorFR, motorBR, motorFL, motorBL, motorLift, motorFlyL, motorFlyR;
    private Servo servoLift1, servoLift2; // also, this goes in port one of the servo controller
    //private Servo servoL, servoR; in case we need to switch back to servos for the flywheels
    /** ------------------------------------------------------------------------------------ **/


    /** Initialize variables for loop() **/
    /** ------------------------------------------------------------------------------------ **/
    double x, y, r = 0;
    double slope, power;

    // Drive motor power variables
    double frontRight, frontLeft, backRight, backLeft;

    double speed = 1.0;
    double change_speed = 0, prev = 0;
    double angle;
    double hypotenuse;

    // Flywheel variables
    double leftFlywheel = 0;
    double rightFlywheel = 0;
    double flywheel = 0.5;

    // Lift variables
    double liftServo = 180;
    static double PLATFORM_LOAD = 0.92;         //0 = up completely, 1 = down completely, 0.8 = flat
    static double PLATFORM_REST = 0.75;
    static double PLATFORM_PLACE = 0.28;
    static double MAX_LIFT_POWER_UP = 0.5;
    static double MAX_LIFT_POWER_DOWN = 0.25;
    double liftPower = 0;

    boolean swap_front_back;
    boolean toggleServo = true;

    boolean gear_ratio_is_07 = true;
    boolean flip_front = false;
    double gearRatio;

    double flyMaxPower = 0.5;                   //The maximum power of the motors for the flywheels (-0.5 to 0.5). Added to making operations easier
    /** ------------------------------------------------------------------------------------ **/




    @Override
    public void init()
    {   /** Assign values to hardware components here (match them to phone configuration)
         *  Motor and motor controller hardware declaration **/
        /** ------------------------------------------------------------------------------------ **/
        //motorControllerDrive = hardwareMap.dcMotorController.get("MC_D");

        motorFL = hardwareMap.dcMotor.get("motorFL");           // Drive motors of the robot
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFlyL = hardwareMap.dcMotor.get("motorFlyL");       // Flywheel motors of the robot
        motorFlyR = hardwareMap.dcMotor.get("motorFlyR");

        motorLift = hardwareMap.dcMotor.get("motorLift");     // Lift motor
        /** ------------------------------------------------------------------------------------ **/


        /** Servo declaration **/
        /** ------------------------------------------------------------------------------------ **/
        //servoFlyL = hardwareMap.servo.get("servoL");          // Flywheel Vex 393 Hybrid Motors (motors that are considered servos
        //servoFlyR = hardwareMap.servo.get("servoR");

        servoLift1 = hardwareMap.servo.get("servoLift1");       // Glyph platform servos
        servoLift2 = hardwareMap.servo.get("servoLift2");       // -> they rotate the platform on the robot that controls the lift
        /** ------------------------------------------------------------------------------------ **/


        /** Encoder stuff - Run Without Encoders is depreciated **/
        /** ------------------------------------------------------------------------------------ **/
        //motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        //motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        /** ------------------------------------------------------------------------------------ **/


        /** Flipping motors (11/10/17) **/
        /** ------------------------------------------------------------------------------------ **/
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);        // Because the motors rotate CCW, the right motors need to be reversed
        motorFL.setDirection(DcMotor.Direction.REVERSE);        // The front motors are reversed because of the gears on the back motors

        motorFlyR.setDirection(DcMotor.Direction.REVERSE);
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        //motorL.setDirection(DcMotorSimple.Direction.REVERSE)
        /** ------------------------------------------------------------------------------------ **/


    }

    public void loop()
    {   /*
         * Gets position of the joysticks on controller1
         * and sets the power of the motors as the position multiplied by a constant
         * to influence the potency of the motors.
         */

        /** Flip Front and Slow Mode
         *  ->  both of these modes modify the robot's movements according to the bumpers on
         *      gamepad 1's controller.
         *      Flip Front is a toggle that reverses the drive motor direction, essentially flipping
         *      the front of the robot
         *      Slow Mode slows down the robot to make it easier to maneuver slowly **/
        /** ------------------------------------------------------------------------------------ **/
        /*
        if (gamepad1.right_bumper)
        {
            gear_ratio_is_07 = !(gear_ratio_is_07);
        }*/
        gearRatio = gamepad1.right_bumper ? 1 : 0.2;
        // If right_bumper toggles gear ratio, the default gearRatio is 0.7. Otherwise, the gearRatio is 0.2

        /*
        if (gamepad1.left_bumper)   // toggles front, kinda unnecessary though
        {
            flip_front = !(flip_front);
        }*/
        /** ------------------------------------------------------------------------------------ **/




        /** Flywheel Code for Gamepad 2 (trigger instead of bumper)
         *      12/8/17 -   depending on the amount of gamepad2 right trigger pushed down,
         *                  both flywheels move according to that multiplied by the maximum
         *                  power value.
         *                  The flywheels suck in when right trigger is pressed and eject when
         *                  left trigger is pressed. **/
        /** ------------------------------------------------------------------------------------ **/
        leftFlywheel = (gamepad2.right_trigger - gamepad2.left_trigger) * flyMaxPower;
        leftFlywheel = Range.clip(leftFlywheel, -1, 1);

        rightFlywheel = leftFlywheel;

        //servoL.setPosition(leftFlywheel);
        //servoR.setPosition(rightFlywheel);

        motorFlyL.setPower(leftFlywheel);
        motorFlyR.setPower(rightFlywheel);
        /** ------------------------------------------------------------------------------------ **/




        /** Lift Code
         *      Includes the glyph platform and lift winch motor codes
         *
         *      Glyph Platform -
         *      - if GP2.Y, then put glyphs into place position
         *      - elif GP2.A, ready platform for loading
         *      - else, hold platform at flat position **/
        /** ------------------------------------------------------------------------------------ **/
        // Determines which lift position to use (default is loading)
        if (gamepad2.y && !gamepad2.a)          //when you press Y on gamepad 2
        {
            liftServo = PLATFORM_PLACE;
        }
        else if (!gamepad2.y && gamepad2.a)     //when you press A on gamepad 2
        {
            liftServo = PLATFORM_LOAD;
        }
        else
        {
            liftServo = PLATFORM_REST;
        }

        liftServo = Range.clip(liftServo, 0.0, 1);

        servoLift1.setPosition(liftServo);
        servoLift2.setPosition(liftServo);


        // Moves the lift motor
        liftPower = gamepad2.left_stick_y;
        if (liftPower < 0)                      // Gamepad2 LStick Y is up
        {
            liftPower *= MAX_LIFT_POWER_UP;
        }
        else                                    // If it's down or 0
        {
            liftPower *= MAX_LIFT_POWER_DOWN;
        }
//        liftPower = liftPower * liftPower * liftPower;
        liftPower = Range.clip(liftPower, -0.5,0.5 );
        motorLift.setPower(liftPower);
        /** ------------------------------------------------------------------------------------ **/




        /** Drive Train Code
         *      Includes both tank drive and Dpad mecanum wheel strafing and full Mecanum movement
         *
         *  Dpad Mecanum Wheel Strafing
            // A modified version ARC Lightning's teleop from last year
            // changes the x and y move values based on which dpad buttons are being held down **/
        /** ------------------------------------------------------------------------------------ **/
        x = y = 0;

        // Dpad Mapping to x and y values
        /* ---------------------------------------------------------- */
        // Horizontal DPad
        if (gamepad1.dpad_right)
        {
            x = -1;
        }
        else if (gamepad1.dpad_left)
        {
            x = 1;
        }
        // Vertical Dpad
        if (gamepad1.dpad_up)
        {
            y = -1;
        }
        else if (gamepad1.dpad_down){
            y = 1;
        }
        /* ---------------------------------------------------------- */

        //We're using tank drive so r isn't really necessary
        //r = gamepad1.right_stick_x; // handles the robot turning

        x = Range.clip(x, -1, 1);
        y = Range.clip(y, -1, 1);

        //We're using tank drive so r isn't really necessary
        //r = Range.clip(r, -1, 1);


        // Speed Control for the movement
        /*
        if (gamepad1.dpad_right && speed < 1)
        {
            change_speed = 1;
        }
        if (gamepad1.dpad_left && speed > 0)
        {
            change_speed = 2;
        }

        if (change_speed != prev)
        {
            if (change_speed == 1)
            {
                speed += 0.1;
            }

            if (change_speed == 2)
            {
                speed -= 0.1;
            }
        }

        prev = change_speed;
        change_speed = 0; */

        // Limits speed to between 0 and 1
        if (speed > 1)
        {
            speed = 1;
        }

        if (speed < 0)
        {
            speed = 0;
        }
        Range.clip(speed, 0, 1);


        /*x *= speed;
        y *= speed;*/

        //We're using tank drive so r isn't really necessary
        //r *= speed;

        /*
        telemetry.addData("x", x);
        telemetry.addData("y", y);

        //We're using tank drive so r isn't really necessary
        //telemetry.addData("r", r);

        telemetry.update(); */

//        x = x * x * x;
//        y = y * y * y;
//        r = r * r * z;

        power = Math.sqrt(x * x + y * y);

        /*
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
        */


        // Finding the angle (IN RADIANS, similar to unit circle)
        // To ensure that undefined is not put into the calculation
        if (gamepad1.left_stick_x == 0)
        {
            if (gamepad1.left_stick_y == 1)
            {
                angle = Math.PI / 2;
            }
            else
            {
                angle = 3 * Math.PI / 2;
            }
        }
        else
        {
            angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
            if (gamepad1.left_stick_x < 0)
            {
                angle += Math.PI;
            }
        }

        // If it's less than 0
        if (angle < 0)
        {
            angle += (2 * Math.PI);
        }

        // If it's somehow becomes more than 2PI
        angle %= 2 * Math.PI;

        // The hypotenuse (essentially the power of the joystick)
        hypotenuse = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);


        // Zone Determination
        /* ---------------------------------------------------------- */
        int zone = -1;
        // 0 is right, 1 is up-right, 2 is up, 3 is up-left,
        // 4 is left, 5 is down-left, 6 is down, 7 is down-right

        if (y == 0) {
            // Right
            if (x > 0) zone = 0;
            // Left
            if (x < 0) zone = 4;
            // Nothing
            if (x == 0) zone = -1;
        }

        if (y > 0)
        {
            // Forward
            if (x == 0) zone = 2;
            // Up-Right
            if (x > 0)
            {
                zone = 1;
            }
            // up-left
            if (x < 0) {
                zone = 3;
            }
        }
        if (y < 0){
            // Backward
            if (x == 0) zone = 6;
            // Down-Right
            if (x > 0)
            {
                zone = 7;
            }
            // down-left
            if (x < 0)
            {
                zone = 5;
            }
        }
        /* ---------------------------------------------------------- */

        power = 1;

        // the two joysticks moving up and down provide tank controls
        /* ---------------------------------------------------------- */
        if (gamepad1.left_stick_y != 0)
        {
            frontLeft = backLeft = gamepad1.left_stick_y;
            zone = -2;
        }

        if (gamepad1.right_stick_y != 0)
        {
            frontRight = backRight = gamepad1.right_stick_y;
            zone = -2;
        }
        /* ---------------------------------------------------------- */

        // Full Mecanum Joystick Quadrant Code
        /* ---------------------------------------------------------- */
        if (gamepad1.left_bumper)
        {
            zone = -3;          // To prevent the DPad from being used
            if (angle == 0)                                                 // If Right
            {
                frontLeft = backRight = hypotenuse;
                frontRight = backLeft = -hypotenuse;
            }
            else if (angle > 0 && angle < (Math.PI / 2))                    // If Quadrant I
            {
                frontLeft = backRight = hypotenuse;
                frontRight = backLeft = -hypotenuse * Math.tan((Math.PI / 4) - angle);
            }
            else if (angle == (Math.PI / 2))                                // If Up
            {
                frontLeft = backRight = hypotenuse;
                frontRight = backLeft = hypotenuse;
            }
            else if (angle > (Math.PI / 2) && angle < Math.PI)              // If Quadrant II
            {
                frontLeft = backRight = hypotenuse * Math.tan((3 * Math.PI / 4) - angle);
                frontRight = backLeft = hypotenuse;
            }
            else if (angle == Math.PI)                                      // If Left
            {
                frontLeft = backRight = -hypotenuse;
                frontRight = backLeft = hypotenuse;
            }
            else if (angle > Math.PI && angle < (3 * Math.PI / 2))          // If Quadrant III
            {
                frontLeft = backRight = -hypotenuse;
                frontRight = backLeft = hypotenuse * Math.tan((5 * Math.PI / 4) - angle);
            }
            else if (angle == (3 * Math.PI / 2))                            // If Down
            {
                frontLeft = backRight = -hypotenuse;
                frontRight = backLeft = -hypotenuse;
            }
            else if (angle > (3 * Math.PI / 2) && angle < (2 * Math.PI))    // If Quadrant IV
            {
                frontLeft = backRight = -hypotenuse * Math.tan((7 * Math.PI / 4) - angle);
                frontRight = backLeft = -hypotenuse;
            }
        }
        /* ---------------------------------------------------------- */

        // Zone Control
        /* ---------------------------------------------------------- */
        if (zone == 0)// right
        {
            frontLeft = backRight = power;
            backLeft = frontRight = -power;
        }
        else if (zone == 4)// left
        {
            frontLeft = backRight = -power;
            backLeft = frontRight = power;
        }

        else if (zone == 2)// up
        {
            frontLeft = backRight = power;
            backLeft = frontRight = power;
        }
        else if (zone == 6)// down
        {
            frontLeft = backRight = -power;
            backLeft = frontRight = -power;
        }

        else if (zone == 1)// up-right
        {
            frontLeft = backRight = power;
            backLeft = frontRight = 0;
        }
        else if (zone == 3)// up-left
        {
            frontLeft = backRight = 0;
            backLeft = frontRight = power;
        }
        else if (zone == 5)// down-left
        {
            frontLeft = backRight = -power;
            backLeft = frontRight = 0;
        }
        else if (zone == 7)// down-right
        {
            frontLeft = backRight = 0;
            backLeft = frontRight = -power;
        }
        else if (zone == -1)//Nothing pressed (stop)
        {
            frontLeft = frontRight = backLeft = backRight = 0;
        }
        /* ---------------------------------------------------------- */



        /*
        if(swap_front_back)
        {
            frontLeft *= -1;
            frontRight *= -1;
            backLeft *= -1;
            backRight *= -1;

            r *= -1;
        }
        */

        // apply rotation (?)
//      /*frontLeft = ((frontLeft) + r) * 0.707;
//      frontRight = ((frontRight) - r) * 0.707;
//      backLeft = ((backLeft) + r) * 0.707;
//      backRight = ((backRight) - r) * 0.707;*/



        frontLeft = Range.clip(frontLeft, -1, 1);
        frontRight = Range.clip(frontRight, -1, 1);
        backLeft = Range.clip(backLeft, -1, 1);
        backRight = Range.clip(backRight, -1, 1);

        // Set Power
        motorFR.setPower(frontRight * gearRatio);
        motorFL.setPower(frontLeft * gearRatio);
        motorBR.setPower(backRight * gearRatio);
        motorBL.setPower(backLeft * gearRatio);
        /** ------------------------------------------------------------------------------------ **/




        /** Add Telemetry data
         *      Yeah.                                                                           **/
        /** ------------------------------------------------------------------------------------ **/
        telemetry.addData("Gear Ratio ", gearRatio);
        //telemetry.addData("Right Power ", rightPower);
        //telemetry.addData("Left Power ", leftPower);
        telemetry.addData("Servo Power", flywheel);
        telemetry.addData("Platform Position", liftServo);

        telemetry.addData("x", x);
        telemetry.addData("y", y);

        telemetry.addData("Speed", speed);

        telemetry.addData("Angle", angle);
        //We're using tank drive so r isn't really necessary
        //telemetry.addData("r", r);

        telemetry.update();
        /** ------------------------------------------------------------------------------------ **/



        /** Some old squeezer code
         *      In case we switch back to the squeezer system                                                                            **/
        /** ------------------------------------------------------------------------------------ **/






         /*rightPower = gearRatio * gamepad1.right_stick_y;
        leftPower = gearRatio * gamepad1.left_stick_y;*/

        /* If the value of the power is lower than the threshold, the robot will set its power to
         the threshold */
        /*double threshold = 0.1;
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
        } */

        // Set the robot's power
       /* motorFR.setPower(rightPower);
        motorBR.setPower(-rightPower); // back motors need to be reversed because of the gears
        motorFL.setPower(leftPower);
        motorBL.setPower(-leftPower); */ // back motors need to be reversed because of the gears

        /*
        final double DEGREE_CHANGER = 0.0001;

        if(gamepad2.right_bumper) // 83.3 is the number of degrees from 0 to snuggly fit the glyph
        {
            squeezePosition += DEGREE_CHANGER;
        }
        if(gamepad2.left_bumper)
        {
            squeezePosition -= DEGREE_CHANGER;
        }

        final double FIT_GLYPH = 82 / 180; // will (probably) always have glyph squeezed
        */


        //while loops should make the clamp gradually open and close
        /*
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
        */

        /*
        // left bumper -> flywheels suck it in
        if(gamepad2.left_bumper)
        {
            leftFlywheel = flywheel;
            rightFlywheel = -flywheel;
        }
       // right bumper -> blows it out
        if(gamepad2.right_bumper)
        {
            leftFlywheel = -flywheel;
            rightFlywheel = flywheel;
        }
        if(!gamepad2.right_bumper && !gamepad2.left_bumper)
        {
            leftFlywheel = 0;
            rightFlywheel = 0;
        }

        */

        // dealing with the motor controlling the lift
        /*
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
        */
        /*
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
        */
        //squeezePosition = -gamepad2.right_trigger + 1;   // defaults to open
        /*
        squeezePosition = gamepad2.right_trigger;          // defaults to closed
        boolean locked = false;                            // whether or not the right bumper has been pressed. Defaults to false.

        if(gamepad2.right_bumper && gamepad2.right_trigger > 0){ //ensures that we don't accidentally lock it open, which could be confusing to the drivers
            locked = !(locked);                            // toggles the squeeze boolean
        }

        if(!locked) {                                      // doesn't update position (locks the position) if the bumper hasn't been pressed
            squeezePosition = Range.clip(squeezePosition, FIT_GLYPH, 1);
            squeeze.setPosition(squeezePosition);
        }
        */
        
    }


}
