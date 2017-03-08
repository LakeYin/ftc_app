package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 *Created by Joshua Krinsky a modification of Lake Yin's code on 11/18/2016.
 */

@TeleOp(name="Encoder", group="TeleOp")

public class Encoder extends OpMode {

    private DcMotorController motorControllerP4;
    private DcMotor launcherMotor;

    @override
    public void init() {
        int distance=1440;
        motorControllerP4 = hardwareMap.dcMotorController.get("MCP4");
        launcherMotor = hardwareMap.dcMotor.get("motorLauncher"); //hardwaremapping the motor for the launcher
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    public void LauncherEncoder {
            boolean launcherMotor = gamepad2.a; //sets the launcher power equal to press A button on the second controller
            
            
    }

    if(gamepad2.a==true)

    {
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setTargetPosition(distance);
        launcherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    else

    {

    }


    {
        telemetry.addData("text", "run at speed");
        telemetry.addData("power", launcherMotor.getPower());
    }
}}
   

