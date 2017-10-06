package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by citruseel on 10/4/2017 with reference to https://www.youtube.com/watch?v=AxKrJEtfuaI.
 * What this essentially does is create our robot's "blueprints" of how we want it's basic code to look like.
    The video referenced z as up and y as forward and back; idk if it's because their phone is flat down with the cameras facing the ceiling and floor
    or if vuforia is actually like that, we will need some testing
    Therefore, paratheses next to coordinates will represent the coords that fit their description
    and the coords outside the parentheses will represent the coords I think should be correct
 */

public class ThunderBot {

    private LinearOpMode myOpMode; //the autonomous OpMode this robot will be running

    private DcMotor motorL = null; //the left motor the robot will be using
    private DcMotor motorR = null; //the right motor the robot will be using

    //These are the robot's target positions which currently are 0 because it's starting
    //tbh, what the coordinates are should'nt really matter here, what matters is axial = forward, lateral = right, yaw = side-rotation
    private double Axial = 0; //Positive is forward in the Z (y)
    private double Lateral = 0; //positive is right in the x, this isn't really useful except for strafing
    private double Yaw = 0; //Positive is CCW in the x-z (x-y) plane

    //Constructor of this object for usage outside of class
    public ThunderBot(){
    }

    public void initRobot(LinearOpMode opMode){
        //So that we can save the references to the hardwaremap in the OpMode
        myOpMode = opMode;

        //What the hardwareMap will store from the names found on the phone's config
        motorR = myOpMode.hardwareMap.dcMotor.get("motorR");
        motorL = myOpMode.hardwareMap.dcMotor.get("motorL");

        //Direction of motors: Forward = CCW
        motorL.setDirection(DcMotor.Direction.REVERSE);
        motorR.setDirection(DcMotor.Direction.FORWARD);

        //Set it so the motors use encoders
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Have the robot reset to rest
        setBot(0,0,0);
    }

    //Sets the robot's position to respectivve axial, lateral, and yaw
    public void setBot(double axial, double lateral, double yaw){
        setAxial(axial);
        setLateral(lateral);
        setYaw(yaw);
        moveBot();
    }

    //this is what we would do if we wanted a TeleOp run through LinearOp mode
    public void manualDrive(){
        setAxial(-myOpMode.gamepad1.left_stick_y); //sets the axial position to what the gamepad's y direction is (forward is -1 so we negate that with a *-1)
        setYaw(-myOpMode.gamepad1.left_stick_x); //sets the Yaw to the gamepad's x, right is positive 1, but since we want the yaw to be negative to represent turn CW we make it so it's negative
    }

    public void moveBot(){
        /*
            Want to move CCW? Yaw > 0
            Want to move CW? Yaw < 0
            Want to move forward? Axial > 0
            Want to move back? Axial < 0
         */
        double left = Yaw - Axial; //whenever bot needs to rotate CCW (forward left or back right), the left must be less than the right
        double right = Yaw + Axial; //whenever bot needs to rotate CW (forward right or back left), the right must be less than the left

        //chooses the maximum value of these two variables
        double max = Math.max(Math.abs(left), Math.abs(right));

        /*makes sure that the maximum won't exceed 1
        If it does, divide the values by the larger of the two so that they proportionally scale down and the greater power is 1*/
        if (max > 1.0){
            left /= max;
            right /= max;
        }

        //finally sets power
        motorL.setPower(left);
        motorR.setPower(right);

        /*This adds telemetry for the positions and for the power
          Note: I think % in telemetry gets the variable mentioned after the quotations respectively
                Not really sure what the +5.2f is though
         */
        myOpMode.telemetry.addData("Axes ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", Axial, Lateral, Yaw);
        myOpMode.telemetry.addData("Power ", "L[%+5.2f], R[%+5.2f]", left, right);


    }

    //Make sure the axial, lateral, and yaw are not too large and stay in range
    public void setAxial(double axial){
        Axial = Range.clip(axial, -1, 1);
    }
    public void setLateral(double lateral){
        Lateral = Range.clip(lateral, -1, 1);
    }
    public void setYaw(double yaw){
        Yaw = Range.clip(yaw, -1, 1);
    }

    //This method makes setting the motor modes simpler
    public void setMode(DcMotor.RunMode mode){
        motorL.setMode(mode);
        motorR.setMode(mode);
    }
}
