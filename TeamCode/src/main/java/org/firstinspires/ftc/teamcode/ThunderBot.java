package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;

/**
 * Created by citruseel on 10/4/2017 with reference to https://www.youtube.com/watch?v=AxKrJEtfuaI.
 * What this essentially does is create our robot's "blueprints" of how we want it's basic code to look like.
    The video referenced z as up and y as forward and back; idk if it's because their phone is flat down with the cameras facing the ceiling and floor
    or if vuforia is actually like that, we will need some testing
    Therefore, parentheses next to coordinates will represent the coords that fit their description
    and the coords outside the parentheses will represent the coords I think should be correct
 */

public class ThunderBot {

    private LinearOpMode myOpMode; //the autonomous OpMode this robot will be running

    private DcMotor frontLeft;                       // Front Left Motor
    private DcMotor backLeft;                       // Back Left Motor
    private DcMotor frontRight;                      // Front Right Motor
    private DcMotor backRight;                      // Back Right Motor

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
        frontLeft = myOpMode.hardwareMap.dcMotor.get("frontLeft");
        frontRight = myOpMode.hardwareMap.dcMotor.get("frontRight");
        backLeft = myOpMode.hardwareMap.dcMotor.get("backLeft");
        backRight = myOpMode.hardwareMap.dcMotor.get("backRight");

        //Direction of motors: Forward = CCW
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

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
        setAxial(-myOpMode.gamepad1.left_stick_x); //sets the axial position to what the gamepad's x direction is (forward is -1 so we negate that with a *-1)
        setYaw(-myOpMode.gamepad1.left_stick_y); //sets the Yaw to the gamepad's y, right is positive 1, but since we want the yaw to be negative to represent turn CW we make it so it's negative
    }

    public void moveBot(){
        /*
            Want to move CCW? Yaw > 0
            Want to move CW? Yaw < 0
            Want to move forward? Axial > 0
            Want to move back? Axial < 0
         */
        double left = Yaw >= 0 ? Yaw + Axial: Yaw - Axial;
        double right = Yaw >= 0 ? Yaw - Axial: Yaw + Axial;

        //chooses the maximum value of these two variables
        double max = Math.max(Math.abs(left), Math.abs(right));

        /*makes sure that the maximum won't exceed 1
        If it does, divide the values by the larger of the two so that they proportionally scale down and the greater power is 1*/
        if (max > 1.0){
            left /= max;
            right /= max;
        }

        //finally sets power
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);

        /*This adds telemetry for the positions and for the power
          Note: I think % in telemetry gets the variable mentioned after the quotations respectively
          http://www.java-samples.com/showtutorial.php?tutorialid=745
                The 5 means that you want to denote 5 spaces in the print
                The .2f represents how many decimal places you want to be printed (in this case 2 out of the already limited 5 spaces)
                f = decmials (probably represents float), g = sig figs, s = characters in string, d = integer, c = character
                + = right justified, - = left justified, no sign = +
         */
        myOpMode.telemetry.addData("Axes ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", Axial, Lateral, Yaw);
        myOpMode.telemetry.addData("Power ", "L[%+5.2f], R[%+5.2f]", left, right);


    }

    public void strafeBot()
    {
        double move_right = Lateral;

        move_right = Range.clip(move_right, -1, 1);

        frontRight.setPower(-move_right);
        backLeft.setPower(-move_right);

        frontLeft.setPower(move_right);
        backRight.setPower(move_right);

        myOpMode.telemetry.addData("Axes ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", Axial, Lateral, Yaw);
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
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backRight.setMode(mode);
        backLeft.setMode(mode);
    }
}
