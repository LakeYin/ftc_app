package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by citruseel on 4/7/2017.
 */
@TeleOp(name = "OmOmBotTelOp", group = "OmOmBot")
public class OmOmBot extends OpMode{
    /**
     * Indicating robot components
     **/
    /* -------------------------------------------------------------------------------------- */
    private DcMotor motorR, motorL, motorF, motorB;

    public void init(){
        /* -------------------------------------------------------------------------------------- */

        motorL = hardwareMap.dcMotor.get("motorL");             // Maps the Left Motor
        motorR = hardwareMap.dcMotor.get("motorR");             // Maps the Right Motor
        motorF = hardwareMap.dcMotor.get("motorF");
        motorB = hardwareMap.dcMotor.get("motorB");

        motorR.setDirection(DcMotorSimple.Direction.REVERSE);   // Reverses Left Motor (so that the
        // robot can go forward)
        motorF.setDirection(DcMotorSimple.Direction.REVERSE);

        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    // Initially sets the motors to run
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    // without encoders, but can be
        motorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop(){
        double constant = 0.5;
        double forwardPower = gamepad1.right_stick_y;
        double sidePower = gamepad1.right_stick_x;
        sidePower = Range.clip(sidePower, -1, 1);
        forwardPower = Range.clip(forwardPower, -1, 1);

        motorF.setPower(sidePower*constant);
        motorB.setPower(sidePower*constant);
        motorL.setPower(forwardPower*constant);
        motorR.setPower(forwardPower*constant);

        telemetry.addData("sidePower", sidePower);
        telemetry.addData("forwardPower", forwardPower);

        telemetry.update();
    }
}
