package org.firstinspires.ftc.teamcode;

/**
 * Created by joshuakrinsky on 4/5/17.
 */
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "SupercellgyroTest", group = "Supercell")
public class gyroTester extends LinearOpMode {

    private DcMotorController MC_M;
    private DcMotor motorR, motorL;

    private DeviceInterfaceModule DIM;
    int zAccumulated;

    public void runOpMode() throws InterruptedException {

        MC_M = hardwareMap.dcMotorController.get("MC_M");       // Maps the Motor Controller

        motorL = hardwareMap.dcMotor.get("motorL");             // Maps the Left Motor
        motorR = hardwareMap.dcMotor.get("motorR");             // Maps the Right Motor

        motorR.setDirection(DcMotorSimple.Direction.REVERSE);   // Reverses Left Motor (so that the
        // robot can go forward)

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);    // Initially sets the motors to run
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double turnSpeed = .15;
        int target = 0;

            GyroSensor gyro;
            ModernRoboticsI2cGyro mrGyro;

            gyro = hardwareMap.gyroSensor.get("gyro");

            mrGyro = (ModernRoboticsI2cGyro) gyro;



            sleep(1000);
            mrGyro.calibrate(); //calibrates the gyro sensor for use and world domination

            waitForStart();

            while (mrGyro.isCalibrating()) { //causes the program to halt while callibration
                // commenses over the rising horizon
            }

            while(opModeIsActive()) {


                zAccumulated = mrGyro.getIntegratedZValue();

                while (Math.abs(zAccumulated - target) > 3) {


                    if (zAccumulated > 0) {
                        motorL.setPower(turnSpeed);
                        motorR.setPower(-turnSpeed);
                    }

                    if (zAccumulated < 0) {
                        motorL.setPower(-turnSpeed);
                        motorR.setPower(turnSpeed);

                    }



                waitOneFullHardwareCycle();

                zAccumulated = mrGyro.getIntegratedZValue();

                telemetry.addData("1. Accumulation", zAccumulated);
                waitOneFullHardwareCycle();

            }
                motorL.setPower(0);
                motorR.setPower(0);
                telemetry.addData("1. Accumulation", zAccumulated);
        }





    }


}