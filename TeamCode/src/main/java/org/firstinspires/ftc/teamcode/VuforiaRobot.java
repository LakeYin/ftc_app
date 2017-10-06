package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by citruseel on 10/4/2017.
 * Here's the OpMode, purposely have not much here. Refernced from https://www.youtube.com/watch?v=AxKrJEtfuaI
 */
@TeleOp(name="Vuforia with Robot Class", group="Vuforia")
public class VuforiaRobot extends LinearOpMode {

    final double TARGET_DISTANCE = 400.0; //how far in mm you want the robot's center to stop at before the target

    //Declaring OpMode members
    ThunderBot robot = new ThunderBot();
    ThunderNavigation nav = new ThunderNavigation();

    @Override
    public void runOpMode(){

        //initiates Robot
        robot.initRobot(this);
        nav.initVuforia(this, robot);

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
            if(nav.areTargetsVisible() && gamepad1.left_bumper){
                nav.setDriving(TARGET_DISTANCE); //sets the target
            }
            else{
                robot.manualDrive();
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
