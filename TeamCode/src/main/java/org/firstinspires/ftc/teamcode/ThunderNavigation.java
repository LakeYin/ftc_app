package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.util.List;

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

import java.util.ArrayList;

import static com.sun.tools.javac.util.Constants.format;

/**
 * Created by citruseel on 10/5/2017.
 * This class essentially just sets up how we want the robot to collect data as it navigates.
 * This DOES NOT tell the robot what to do.
 *refernce to https://www.youtube.com/watch?v=AxKrJEtfuaI
 */

public class ThunderNavigation {
    //constants we want to never change
    private static final int MAX_TARGETS = 4;
    private static final double ON_AXIS = 10; //used to check to see if it's within "x"mm of target's center line/on the x coord of 0
    private static final double CLOSE_ENOUGH = 20; //used to check to see if it's within "x"mm of target to stop moving

    //selecting which camera on phone
    private static final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.BACK;

    //The bigger the gain the more responsive (actions will be done more quickly)
    private static final double YAW_GAIN = 0.018; //rate at which we respond to angling error
    private static final double LATERAL_GAIN = 0.0027; //rate at responding to off-axis error (strafing mainly)
    private static final double AXIAL_GAIN = 0.0017; //rate at responding target distance error

    private LinearOpMode myOpMode;
    private ThunderBot myRobot;
    private VuforiaTrackables targets;

    //Some data we want to keep track of as we run the OpMode
    private boolean targetFound;
    private String targetName;
    private double robotX; //X displacement from target; right is positive
    private double robotZ; //Z displacement from target; z is negative and closer to z would be 0
    private double robotBearing; //robot rotation around the Z axis CCW = + like Yaw basically
    private double targetRange; //how far target is mm
    private double targetBearing; //direction of target from center ignoring the robot's orientation
    private double relativeBearing; //Bearing change needed to be achieved (positive means robot must turn CCW to face target)

    //Constructor: how we want the navigation to start out when referenced in other classes
    public ThunderNavigation() {
        targetFound = false;
        targetName = null;
        targets = null;

        robotX = 0;
        robotZ = 0;
        targetRange = 0;
        targetBearing = 0;
        robotBearing = 0;
        relativeBearing = 0;
    }

    public void addNavTelemetry() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();           // OR... Use this line to improve performance

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters); //make a vuforia localizer (basically initiates phone)

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        //tells us what type of vumark is seen by the phone's camera
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            myOpMode.telemetry.addData("VuMark", "%s is visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
            myOpMode.telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = -1* trans.get(0);
                double tY = trans.get(1);
                double tZ = -1* trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
        }
        if(targetFound){
            myOpMode.telemetry.addData("Visible", targetName); //tells us the targets
            myOpMode.telemetry.addData("Robot", " X[%5.0fmm] Z[%5.0fmm] B(%4.0f°)", robotX, robotZ, robotBearing);// tells us the robots x, y, and angle
            myOpMode.telemetry.addData("Target", "R[%5.0fmm] B(%4.0f°) RB(%4.0f°)", targetRange, targetBearing, relativeBearing);// tells us the target's distance, angle, and the angle we need to turn
            myOpMode.telemetry.addData("Turn", "%s %4.0f°",  relativeBearing < 0 ? ">>> CW " : "<<< CCW", Math.abs(relativeBearing)); //basically tells us how much we need to turn
            myOpMode.telemetry.addData("X Distance", "%5.0fmm", Math.abs(robotX)); //tells us the x distance away
        }
        else{
            myOpMode.telemetry.addData("Visible", "N/A"); //can't see targets
        }
    }

    //activates targets if found
    public void startTracking(){
        if(targets != null){
            targets.activate();
        }
    }

    //This is where the fun stuff is: How does the robot calculate the path
    public boolean setDriving(double targetrange) {
        boolean closeEnough;

        //The math here I don't really get but whatever, we can analyze this together later if we want to really truly understand it
        //He mentions that you multiply the error by the gain, and its not PID but proportional if that helps
        //My take on it is that it takes a proportion of the error and fixes it, and then takes a proportion of that error and fixes it, etc.
        // Priority #1 Rotate to always be pointing at the target (for best target retention).
        double Yaw  = (relativeBearing * YAW_GAIN);

        // Priority #2  Drive laterally based on distance from X axis (same as z value)
        double Lateral  = (robotZ * LATERAL_GAIN);

        // Priority #3 Drive forward based on the angle to the target standoff distance
        double Axial  = (-(robotX + targetrange) * AXIAL_GAIN);

        // Set the desired positions
        myRobot.setYaw(Yaw);
        myRobot.setAxial(Axial);
        myRobot.setLateral(Lateral);

        // Determine if we are close enough to the target for action.
        closeEnough = ( (Math.abs(robotX + targetrange) < CLOSE_ENOUGH) &&
                (Math.abs(robotZ) < ON_AXIS));

        return (closeEnough); //update boolean
    }

    //Init vuforia with the camera direction and create a matriz to tell vufori where everything is
    public void initVuforia(LinearOpMode opMode, ThunderBot robot) {

        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        myRobot = robot;

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);  // Use this line to see camera display
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();                             // OR... Use this line to improve performance

        //just license key we got free online
        parameters.vuforiaLicenseKey = "AQRacK7/////AAAAGea1bsBsYEJvq6S3KuXK4PYTz4IZmGA7SV88bdM7l26beSEWkZTUb8H352Bo/ZMC6krwmfEuXiK7d7qdFkeBt8BaD0TZAYBMwHoBkb7IBgMuDF4fnx2KiQPOvwBdsIYSIFjiJgGlSj8pKZI+M5qiLb3DG3Ty884EmsqWQY0gjd6RNhtSR+6oiXazLhezm9msyHWZtX5hQFd9XoG5npm4HoGaZNdB3g5YCAQNHipjTm3Vkf71rG/Fffif8UTCI1frmKYtb4RvqiixDSPrD6OG6YmbsPOYUt2RZ6sSTreMzVL76CNfBTzmpo2V0E6KKP2y9N19hAum3GZu3G/1GEB5D+ckL/CXk4JM66sJw3PGucCs";

        //set camera direction
        parameters.cameraDirection = CAMERA_DIRECTION;
        parameters.useExtendedTracking = false; //this is just if we want to track past the targets but i think it's just a waste of energy for our robot
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters); //make a vuforia localizer (basically initiates phone)

        //basically gets the targets
        //kinda strange tho that there's only one, will have to look into this
        targets = vuforia.loadTrackablesFromAsset("RelicVuMark");
        targets.get(0).setName("RelicRecovery");

        // create a matrix of the field with the images
        //I think this is for red only? said so in the video
        OpenGLMatrix targetOrientation = OpenGLMatrix
                .translation(0, 150, 0) //put the center of the image 6" above the origin of the field
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, //type of rotation (Extrinsic = coordinate system doesn't rotate, intrisic = coordinate system does rotate along with the object's rotation). Axes order (what the angles will be)
                        AngleUnit.DEGREES, 90, -90, 0)); // rotate images so they aren't flat and are basically facing our right side.

        //we're going to need to get these values and measure them out
        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // Camera is ON the robots center line

        //Matrix of field with phone's location on it
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT, CAMERA_FORWARD_DISPLACEMENT) //for some reason the original coder had the order of forward(x), left(y), and then vertical (z). I think that's just because their phone was lying flat down in landscape with the top to the left
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 0, 0, 0));
    }

    //checks to find a target
    public boolean areTargetsVisible()  {

        int targetTestID = 0;

        // Check each target in turn, but stop looking when the first target is found.
        while ((targetTestID < MAX_TARGETS) && !targetIsVisible(targetTestID)) {
            targetTestID++ ;
        }

        return (targetFound);
    }

    public boolean targetIsVisible(int targetId) {

        VuforiaTrackable target = targets.get(targetId);
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        OpenGLMatrix location  = null; //this will be updated in the program by the code

        // if we have a target, look for an updated robot position
        if ((target != null) && (listener != null) && listener.isVisible()) {
            targetFound = true;
            targetName = target.getName();

            // This stuff only happens when new data is discovered
            location  = listener.getUpdatedRobotLocation();
            if (location != null) {

                // Create a translation and rotation vector for the robot.
                VectorF trans = location.getTranslation();
                Orientation rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Robot position in matrix x = 0, y = 1, z = 2
                robotX = -1* trans.get(0);
                robotZ = -1* trans.get(2);

                // Robot bearing is defined by the standard Matrix z rotation
                //basically the bearing = the rotation angle
                robotBearing = rot.thirdAngle;

                // Target range calculated from pythagorean theorem
                targetRange = Math.hypot(robotX, robotZ);

                // target bearing is based on angle formed between the X axis and the target range line
                targetBearing = Math.toDegrees(-Math.asin(robotZ / targetRange));

                // Target relative bearing is the target bearing relative to the direction the robot is pointing.
                relativeBearing = targetBearing - robotBearing;
            }

            //at this point i think it is safe to say that we have, indeed, found a target
            targetFound = true;
        }
        else  {// if no target visible
            targetFound = false;
            targetName = "None";
        }

        return targetFound; //update boolean
    }
}
