package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

/**
 * Created by citruseel on 12/6/2017.
 */
@TeleOp(name="SingleJoystickTestBench", group="TestBench")
public class TestBotSingleJoystick extends OpMode {

    private DcMotor motorL;
    private DcMotor motorR;

    public void init(){

        //What the hardwareMap will store from the names found on the phone's config
        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");

        //Direction of motors: Forward = CCW
        motorL.setDirection(DcMotor.Direction.REVERSE);

        //Set it so the motors use encoders
        motorL.setMode(RUN_WITHOUT_ENCODER);
        motorR.setMode(RUN_WITHOUT_ENCODER);
    }

    public void loop(){
        double joystick_x = -1* gamepad1.left_stick_x;
        double joystick_y = -1* gamepad1.left_stick_y;

        double left = joystick_y >= 0 ? joystick_y + joystick_x : joystick_y - joystick_x;
        double right = joystick_y >= 0 ? joystick_y - joystick_x : joystick_y + joystick_x;

        //chooses the maximum value of these two variables
        double max = Math.max(Math.abs(left), Math.abs(right));

        /*makes sure that the maximum won't exceed 1
        If it does, divide the values by the larger of the two so that they proportionally scale down and the greater power is 1*/
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        motorL.setPower(left);
        motorR.setPower(right);

        telemetry.addData("Power ", "L[%+5.2f], R[%+5.2f]", left, right);

    }
}