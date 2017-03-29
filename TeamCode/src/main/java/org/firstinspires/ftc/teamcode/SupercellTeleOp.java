package org.firstinspires.ftc.teamcode;

/**
 * Created by Alex on 3/14/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.Range;

/**
 * This program is a basic teleop for our robot testbed, ARC Supercell **/

@TeleOp(name = "Supercell TeleOp", group = "Supercell")
public class SupercellTeleOp extends OpMode {

    /** Indicating robot components **/
    /* -------------------------------------------------------------------------------------- */
    private DcMotorController MC_M;
    private DcMotor motorR, motorL;

    private DeviceInterfaceModule DIM;
    /* -------------------------------------------------------------------------------------- */


    @Override
    public void init() {
        /* Initializing and mapping electronics */

        /** Drive Motors and Respective MC **/
        /* -------------------------------------------------------------------------------------- */
        MC_M = hardwareMap.dcMotorController.get("MC_M");       // Maps the Motor Controller

        motorL = hardwareMap.dcMotor.get("motorL");             // Maps the Left Motor
        motorR = hardwareMap.dcMotor.get("motorR");             // Maps the Right Motor

        motorR.setDirection(DcMotorSimple.Direction.REVERSE);   // Reverses Left Motor (so that the
                                                                // robot can go forward)

        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    // Initially sets the motors to run
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    // without encoders, but can be
                                                                // changed later in the code.
        /* -------------------------------------------------------------------------------------- */

        /** DIM and Sensors **/
        /* -------------------------------------------------------------------------------------- */
        DIM = hardwareMap.deviceInterfaceModule.get("DIM");     // Maps the Device Interface Module
        /* -------------------------------------------------------------------------------------- */
    }

    public void loop() {

        /** For the Drive Train **/
        /* -------------------------------------------------------------------------------------- */
        double speedModifier = gamepad1.right_trigger > 0 ? 0.25: gamepad1.right_bumper ? 1 : 0.6;
            // Modifies the power (below) depending on the right trigger and bumper

        double leftPower = gamepad1.left_stick_y * speedModifier;     // Both set power according to
        double rightPower = gamepad1.right_stick_y * speedModifier;   // the joysticks on controller 1

        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        motorL.setPower(leftPower);                             // Sets the motor power equal to
        motorR.setPower(rightPower);                            // each's respective power

        telemetry.addData("Left Motor Power", leftPower);       // Adds telemetry
        telemetry.addData("Right Motor Power", rightPower);
        /* -------------------------------------------------------------------------------------- */


        telemetry.update();                                     // Updates telemetry
    }

}
