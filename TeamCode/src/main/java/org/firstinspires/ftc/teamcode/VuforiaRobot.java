package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by citruseel on 10/4/2017.
 * Here's the OpMode, purposely have not much here. Refernced from https://www.youtube.com/watch?v=AxKrJEtfuaI
 */
@TeleOp(name="Vuforia with Robot Class", group="Vuforia")
public class VuforiaRobot extends AutonomousMethodMaster {

    private DcMotor motorL;                       // Left Side Motor
    private DcMotor motorR;                       // Right Side Motor

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.875 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    final double TARGET_DISTANCE = 400.0; //how far in mm you want the robot's center to stop at before the target

    //Declaring OpMode members
    ThunderBot robot = new ThunderBot(); // Uses Thunder hardware
    ThunderNavigation nav = new ThunderNavigation();

    @Override
    public void runOpMode(){

        //initiates Robot
        robot.initRobot(this);
        nav.initVuforia(this, robot);

        // maps the drive motors
        motorL = hardwareMap.dcMotor.get("motorL");        //P0 is actually the right
        motorR = hardwareMap.dcMotor.get("motorR");        //P1 is actually the left

        //resets encoders and tells the motors to run with them
        encoderMode(3);
        encoderMode(1);

        //start the tracking my friends
        nav.startTracking();

        //basically tells us to start it when not started
        while (!isStarted()){
            telemetry.addData(">","Start me");

            nav.areTargetsVisible();
            nav.addNavTelemetry();

            telemetry.update();
        }

        //where all the good stuff happens
        while (opModeIsActive()){

            //made so that it starts tracking when you press the left bumper, otherwise manual drive which I wanted to test to see if it works (single joystick?)
            telemetry.addData(">", "Press Left Bumper to track target");
            if(!nav.areTargetsVisible() && gamepad1.left_bumper){
                robot.manualDrive();
            }
            else{
                nav.setDriving(TARGET_DISTANCE); //sets the target
            }

            //update nav telem
            nav.addNavTelemetry();

            //at the end of every cycle, move the robot and update telemetry
            robot.moveBot();
            telemetry.update();
        }

        telemetry.addData(">", "Finished");
        telemetry.update();
    }
}
