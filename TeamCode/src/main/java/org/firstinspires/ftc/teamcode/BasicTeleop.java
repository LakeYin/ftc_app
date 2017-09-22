package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by user on 9/22/2017.
 */

@Disabled
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

        // Encoder stuff
        /* ---------------------------------------- */
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        //motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        /* ---------------------------------------- */

    }

    public void loop()
    {   // Do stuff here
        double rightPower, leftPower;
        rightPower = gamepad1.left_stick_y;
        leftPower = gamepad1.right_stick_y;

        motorR.setPower(rightPower);
        motorL.setPower(leftPower);
    }
}