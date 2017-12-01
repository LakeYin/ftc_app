package org.firstinspires.ftc.teamcode;

/**
 * Created by justin on 12/1/17.
 */

public class DraftAutoVuforiaL1 extends AutonomousMethodMaster{

    public void runOpMode()
    {
        initElectronics(0);

        waitForStart();

        // red1
        encoderStrafeRight(1, 24);

        int move_inches = 0;
        // identify which vumark

        encoderStrafeRight(1, move_inches);

        encoderMove(1, 12, 12);

        // place glyph
    }

}
