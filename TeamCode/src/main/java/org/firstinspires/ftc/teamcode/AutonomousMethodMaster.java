package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

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

import java.lang.Math;

/**
 * Created by Alex on 9/18/2017.
 *  *
 * NOTE:
 *
 * This is intended to be the master autonomous file. Place all methods that you write into this
 * file so they can be inherited by other autonomous files. By using this approach, we can increase
 * the readability of autonomous files and have a database of various methods so they do not have
 * to be written down in each one.
 */

@Disabled
@Autonomous(name = "Method Master", group = "Master")
public class AutonomousMethodMaster extends LinearOpMode {

    /** Declaring the motor variables **/
    /** ---------------------------------------------------------------------------------------- **/

    private DcMotor motorFR, motorBR, motorFL, motorBL, motorLift, motorFlyL, motorFlyR;
    private Servo servoLift1, servoLift2; // also, this goes in port one of the servo controller

    /** ---------------------------------------------------------------------------------------- **/
    /** For Encoders and specific turn values **/
    /* ------------------------------------------------------------------------------------------ */
    double ticksPerRevNeverest40 = 1120;            // This is the specific value for NeveRest 40s
    double ticksPerRevNeverest20 = 560;             // The specific value for NeveRest 20s
    double ticksPerRevTetrix = 1440;                // The specific value for Tetrix, since only one encoded Tetrix motor (launcher arm)
    double ticksPer360Turn = 4600;                  // The amount of ticks for a robot 360 degree turn (AndyMark NeveRest 40s)
    double tickTurnRatio = ticksPer360Turn / 360;   // The ratio for multiplication
    double inchToMm = 25.4;                         // For conversion between the vectors

    double wheelDiameter = 4.5;                     // Diameter of the current mecanum wheels in inches
    double ticksPerInchNeverest40 = (ticksPerRevNeverest40 / (wheelDiameter * Math.PI));        // The number of encoder ticks per inch (specific to NeveRest 40s
    static double PLATFORM_LOAD = 0.92; //0 = up completely, 1 = down completely, 0.8 = flat
    static double PLATFORM_REST = 0.8;
    static double PLATFORM_PLACE = 0.2;
    ColorSensor colorSensor;
    HiTechnicNxtGyroSensor NxtGyroSensor;
    /* ------------------------------------------------------------------------------------------ */


    public void runOpMode() throws InterruptedException {
        /** This is the method that executes the code and what the robot should do **/
        // Call any variables not stated before

        // Initializes the electronics
        initElectronics(0);

        telemetry.addData("Phase 1", "Init");
        telemetry.update();

        waitForStart();

        telemetry.addData("Started Robot", "Now");
        telemetry.update();

        encoderMode(1);

    }

    /**
     * These methods control the encoder modes of the motor
     *
     * Mode Numbers:
     *  0 = RUN_TO_POSITION
     *  1 = RUN_USING_ENCODER
     *  2 = RUN_WITHOUT_ENCODER
     *  3 = STOP_AND_RESET_ENCODERS
     *  4 = RESET_ENCODERS
     **/
    /** ----------------------------------------- **/
    public void encoderMode(int mode) {
        /**NOTE:
         *  This was made just for the sake of making the code look a bit neater
         *
         * Mode Numbers:
         *  0 = RUN_TO_POSITION
         *  1 = RUN_USING_ENCODER
         *  2 = RUN_WITHOUT_ENCODER
         *  3 = STOP_AND_RESET_ENCODERS
         *  4 = RESET_ENCODERS
         *  **/
        if (mode == 0) {
            /** Sets the encoded motors to RUN_TO_POSITION **/
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (mode == 1) {
            /** Sets the encoders to RUN_USING_ENCODERS **/
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (mode == 2) {
            /** Sets the encoders to RUN_WITHOUT_ENCODERS **/
            motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (mode == 3) {
            /** Stops and resets the encoder values on each of the drive motors **/
            motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if (mode == 4) {
            /** Resets the encoder values on each of the drive motors **/
            motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);
            motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);
            motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
            motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        }
    }

    public void addTelemetryData(String string1, String string2) {
        telemetry.addData(string1, string2);
        telemetry.update();
    }

    public void stopMotion() {
        /** Stops all drive motor motion **/
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }
    /** ----------------------------------------- **/


    /**
     * These methods are used to set up the robot
     **/

    /** ----------------------------------------- **/
    public void initElectronics(int mode){
        // To make the initialization of electronics much easier and nicer to read
        /** Initializing and mapping electronics **/
        if (mode == 0) {
            //Assign values to hardware components here (match them to phone configuration)
            // Motor and motor controller hardware declaration
        /* ---------------------------------------- */
            //motorControllerDrive = hardwareMap.dcMotorController.get("MC_D");

            motorFL = hardwareMap.dcMotor.get("motorFL");
            motorBL = hardwareMap.dcMotor.get("motorBL");
            motorFR = hardwareMap.dcMotor.get("motorFR");
            motorBR = hardwareMap.dcMotor.get("motorBR");
        /* ---------------------------------------- */

            // Encoder stuff - Run Without Encoders is depreciated
        /* ---------------------------------------- */
            encoderMode(1);

            // Flipped the motors (11/10/17)
            motorFL.setDirection(DcMotor.Direction.REVERSE);
            motorBR.setDirection(DcMotor.Direction.REVERSE);
            //motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        /* ---------------------------------------- */

            motorFlyL = hardwareMap.dcMotor.get("motorFlyL");       // Flywheel motors of the robot
            motorFlyR = hardwareMap.dcMotor.get("motorFlyR");

            motorLift = hardwareMap.dcMotor.get("motorLift");     // Lift motor

            servoLift1 = hardwareMap.servo.get("servoLift1");       // Glyph platform servos
            servoLift2 = hardwareMap.servo.get("servoLift2");       // -> they rotate the platform on the robot that controls the lift

            //squeeze = hardwareMap.servo.get("squeeze");
            //motor_lift = hardwareMap.dcMotor.get("lift");
        }
    }


    public void encoderMove(double power, double leftInches, double rightInches) {
        /** This method makes the motors move a certain distance **/
        // Set the encoder mode to 3 (STOP_AND_RESET_ENCODERS)
        encoderMode(3);

        // Sets the power range
        power = Range.clip(power, -1, 1);

        // Setting the target positions
        motorFL.setTargetPosition((int)(leftInches * -ticksPerInchNeverest40));
        motorBL.setTargetPosition((int)(leftInches * -ticksPerInchNeverest40));
        motorFR.setTargetPosition((int)(rightInches * -ticksPerInchNeverest40));
        motorBR.setTargetPosition((int)(rightInches * -ticksPerInchNeverest40));

        // Set encoder mode to RUN_TO_POSITION
        encoderMode(0);

        // Sets the motors' position
        motorFL.setPower(power);
        motorBL.setPower(power);
        motorFR.setPower(power);
        motorBR.setPower(power);

        // While loop for updating telemetry
        while(motorFL.isBusy() && motorFR.isBusy() && opModeIsActive()){

            // Updates the position of the motors
            double LPos = motorFL.getCurrentPosition();
            double RPos = motorFR.getCurrentPosition();

            // Adds telemetry of the drive motors
            telemetry.addData("motorFL Pos", LPos);
            telemetry.addData("motorFR Pos", RPos);

            // Updates the telemetry
            telemetry.update();

        }

        // Stops the motors
        stopMotion();

        // Resets to run using encoders mode
        encoderMode(1);

    }

    public void encoderStrafeRight(double power, double rightInches)
    {

        // Set the encoder mode to 3 (STOP_AND_RESET_ENCODERS)
        encoderMode(3);

        // Sets the power range
        power = Range.clip(power, -1, 1);

        // Setting the target positions
        motorFL.setTargetPosition((int)(rightInches * -ticksPerInchNeverest40));
        motorBL.setTargetPosition((int)(rightInches * ticksPerInchNeverest40));
        motorFR.setTargetPosition((int)(rightInches * ticksPerInchNeverest40));
        motorBR.setTargetPosition((int)(rightInches * -ticksPerInchNeverest40));

        // Set encoder mode to RUN_TO_POSITION
        encoderMode(0);

        motorFR.setPower(power);
        motorBL.setPower(power);
        motorFL.setPower(power);
        motorBR.setPower(power);

        // While loop for updating telemetry
        while(motorFL.isBusy() && motorFR.isBusy() && opModeIsActive()){

            // Updates the position of the motors
            double LPos = motorFL.getCurrentPosition();
            double RPos = motorFR.getCurrentPosition();

            // Adds telemetry of the drive motors
            telemetry.addData("motorFL Pos", LPos);
            telemetry.addData("motorFR Pos", RPos);

            // Updates the telemetry
            telemetry.update();

        }

        // Stops the motors
        stopMotion();

        // Resets to run using encoders mode
        encoderMode(1);
    }


    public void encoderRotateDegrees(int direction, double power, int robotDegrees) {

        /** This method, given an input amount of degrees, makes the robot turn
         *  the amount of degrees specified around ITS center of rotation **/
        encoderMode(3);

        // Sets the power range
        power = Range.clip(power, -1, 1);
        power = Math.abs(power);

        // Setting variables
        double robotTurn = robotDegrees * tickTurnRatio;

        // Setting the target positions
        if (direction == 1)
        { //counterclockwise (left)
            motorFL.setTargetPosition((int)(robotTurn));
            motorBL.setTargetPosition((int)(robotTurn));
            motorFR.setTargetPosition((int)(-robotTurn));
            motorBR.setTargetPosition((int)(-robotTurn));
        }
        else
        { //clockwise (right)
            motorFL.setTargetPosition((int)(-robotTurn));
            motorBL.setTargetPosition((int)(-robotTurn));
            motorFR.setTargetPosition((int)(robotTurn));
            motorBR.setTargetPosition((int)(robotTurn));
        }

        encoderMode(0);

        // Sets the motors' positions
        motorFL.setPower(power);
        motorBL.setPower(power);
        motorFR.setPower(power);
        motorBR.setPower(power);

        // While loop for updating telemetry
        while(motorFL.isBusy() && motorFR.isBusy() && opModeIsActive())
        {
            // Updates the position of the motors
            double LPos = motorFL.getCurrentPosition();
            double RPos = motorFR.getCurrentPosition();

            // Adds telemetry of the drive motors
            telemetry.addData("motorFL Pos", LPos);
            telemetry.addData("motorFR Pos", RPos);

            // Updates the telemetry
            telemetry.update();

        }

        // Stops the motors
        stopMotion();

        // Resets to run using encoders mode
        encoderMode(1);
    }

    VuforiaLocalizer vuforia;

    public void VuforiaSetup(){
        // set up all of the vuforia
        // tells vuforia to use the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // the licence key that vuforia needs to work
        parameters.vuforiaLicenseKey = "AQRacK7/////AAAAGea1bsBsYEJvq6S3KuXK4PYTz4IZmGA7SV88bdM7l26beSEWkZTUb8H352Bo/ZMC6krwmfEuXiK7d7qdFkeBt8BaD0TZAYBMwHoBkb7IBgMuDF4fnx2KiQPOvwBdsIYSIFjiJgGlSj8pKZI+M5qiLb3DG3Ty884EmsqWQY0gjd6RNhtSR+6oiXazLhezm9msyHWZtX5hQFd9XoG5npm4HoGaZNdB3g5YCAQNHipjTm3Vkf71rG/Fffif8UTCI1frmKYtb4RvqiixDSPrD6OG6YmbsPOYUt2RZ6sSTreMzVL76CNfBTzmpo2V0E6KKP2y9N19hAum3GZu3G/1GEB5D+ckL/CXk4JM66sJw3PGucCs";

        // indicates camera direction
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // loads data for vumarks
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();
    }

    // added a new sleep method because the default one isn't working
    public static void sleepNew(long sleepTime)
    {
        long wakeupTime = System.currentTimeMillis() + sleepTime;

        while (sleepTime > 0)
        {
            try
            {
                Thread.sleep(sleepTime);
            }
            catch (InterruptedException e)
            {
                sleepTime = wakeupTime - System.currentTimeMillis();
            }
        }
    }

    // method to dump the glyphs during autonomous
    public void dumpGlyph()
    {
        // tells the servos to dump the glyph
        servoLift1.setPosition(PLATFORM_PLACE);
        servoLift2.setPosition(PLATFORM_PLACE);

        // sleep for 1 second
        sleep(1000);

        // restore platform to resting position
        servoLift1.setPosition(PLATFORM_REST);
        servoLift2.setPosition(PLATFORM_REST);
    }
    /** ----------------------------------------- **/
        /*
    * colour Sensor code set up Josh K.
    *
    *
    * */
        public void setUpColourSensor()
        {
            // bLedOn represents the state of the LED.
            boolean bLedOff = false;

            // get a reference to our ColorSensor object.
            colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

            // Set the LED off in the beginning
            colorSensor.enableLed(bLedOff);
        }

    /** ----------------------------------------- **/

    /** ----------------------------------------- **/
        /*
    * GyroScope code set up Josh K.
    *
    *
    * */
    public void setUpGyroScopeHT()
    {
        //gets the refrence to the hardware for the NxtSensor as well
        NxtGyroSensor = hardwareMap.get(HiTechnicNxtGyroSensor.class, "gyro");

        //giving time to calibrate the gryroscope
        telemetry.log().add("calibrating...");
        NxtGyroSensor.calibrate(3000, 100);
        telemetry.log().add("...done...waiting for start...");

    }

/*
    public void runColorMode() {

        float hsvValues[] = {0F, 0F, 0F};


        //holds HSV values within an array
        final float values[] = hsvValues;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensorColor");

        // bLedOn represents the state of the LED.
        boolean bLedOn = false;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // wait for the start button to be pressed.
        waitForStart();


        while (opModeIsActive()) {


            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayou+
                .setBackgroundColor(Color.WHITE);
            }
        });
    }
    */
}
