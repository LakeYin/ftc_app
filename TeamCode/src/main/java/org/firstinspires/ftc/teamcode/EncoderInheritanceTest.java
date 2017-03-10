package org.firstinspires.ftc.teamcode;

/**
 * Created by Alex on 3/10/2017.
 */

public class EncoderInheritanceTest extends EncoderAutoBasicMethods{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        encoderMove(0.2, 6, 6);
    }
}
