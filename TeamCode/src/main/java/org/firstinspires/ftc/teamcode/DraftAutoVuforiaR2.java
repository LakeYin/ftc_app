package org.firstinspires.ftc.teamcode;

/**
 * Created by justin on 12/1/17.
 */

public class DraftAutoVuforiaR2 extends AutonomousMethodMaster{

    public void runOpMode()
    {
        initElectronics(0);

        waitForStart();

        // red1
        encoderStrafeRight(1, -24);

        int move_inches = 0;
        // identify which vumark

        encoderRotateDegrees(1, 1, 90);

        encoderStrafeRight(1, -move_inches);

        encoderMove(1, 12, 12);

        // place glyph
    }

}
