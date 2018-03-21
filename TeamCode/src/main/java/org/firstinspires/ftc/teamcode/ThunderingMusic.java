package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * A demonstration of patronage through implementation of an actuator-based music player for our
 * State-Qualified fellow team, 5273 ARC Thunder.
 *
 * @author Michael Peng
 * For team: 5273 (Thunder)
 *
 * FIRST - Gracious Professionalism
 */
@Autonomous(name = "SickBeats", group = "Pragmaticos")
public class ThunderingMusic extends LinearOpMode {

    // The instrument instances
    private PlatformInstrument  platform;
    private MotorInstrument     flywheelLeft;
    private MotorInstrument     flywheelRight;

    // The parameters
    private final int BPM = 300;

    // The soundtracks
    private final HashMap<Instrument, Note[]> tracks = new HashMap<>();

    @Override
    public void runOpMode() {
        // Define the instruments
        platform        = new PlatformInstrument("servoLift1", 1.0, 0.8, 200);
        flywheelLeft    = new MotorInstrument("motorFlyL", 800);
        flywheelRight   = new MotorInstrument("motorFlyR", 800);

        // Define the soundtracks
        // NOTE that the soundtracks shouldn't be defined before the instruments.
        tracks.put(platform, new Note[] {
                new Note(2),
                new Note(1),
                new Note(1),
                new Note(2),
                new Note(4)
        });
        tracks.put(flywheelLeft, new Note[] {
                new Note(523.25, 2),
                new Note(2),
                new Note(440, 2),
                new Note(4),
                new Note(493.88, 2),
                new Note(523.25, 2)
        });
        tracks.put(flywheelRight, new Note[] {
                new Note(2),
                new Note(392, 1),
                new Note(392, 1),
                new Note(2),
                new Note(392, 4),
                new Note(493.88, 2),
                new Note(523.25, 2)
        });

        // Enlighten the user
        telemetry.addLine("Welcome to ThunderingMusic!");
        telemetry.addLine("The application is ready to 'make some noise.' Here are some stats:");
        telemetry.addData("BPM", BPM);
        telemetry.addData("Total track count", tracks.size());

        telemetry.update();

        // Control the impulsiveness
        waitForStart();

        // Initialize the threads
        Set<Thread> threads = new HashSet<>();

        for (Map.Entry<Instrument, Note[]> track : tracks.entrySet()) {
            threads.add(new Thread(new TrackExecutor(track.getKey(), track.getValue())));
        }

        // Start the threads
        for (Thread thread : threads) {
            thread.start();
        }

        // Wait for the threads
        try {
            for (Thread thread : threads) {
                thread.join();
            }
        } catch (InterruptedException exc) {
            telemetry.addData("OOPS!", "TrackExecutor thread got interrupted");
            telemetry.addData("Exception Info", exc.getCause());
            telemetry.update();
        }
    }

    // The instruments

    interface Instrument {
        void play(double pitch, int ms);
    }

    // The Implementations

    class PlatformInstrument implements Instrument {
        /**
         * The position of the platform servo where the platform hits the floor.
         */
        private double posDown = 1.0;

        /**
         * The position of the platform servo where the platform is flat.
         */
        private double posUp = 0.8;

        /**
         * The time it takes for the platform servo to move from DOWN/UP to UP/DOWN.
         *
         * Unit: millisecond
         */
        private int moveDelay = 100;

        // ---

        private Servo actuator;

        PlatformInstrument(String deviceName,
                           double downPos,
                           double upPos,
                           int moveDelay) {
            actuator = hardwareMap.servo.get(deviceName);
            posDown = downPos;
            posUp = upPos;
            this.moveDelay = moveDelay;
        }

        @Override
        public void play(double pitch, int ms) {
            // Actuate servo
            actuator.setPosition(posDown);

            // Allow platform to hit
            sleep(moveDelay);

            // Raise the platform
            actuator.setPosition(posUp);

            // Allow platform to raise, then wait for given amount (with corrections)
            sleep(ms - moveDelay);

            // The beats too fast for our instrument?
            if (ms - moveDelay < moveDelay) {
                Log.w("ThunderingMusic",
                        String.format("Beats too quick! moveDelay=%d beats=%d", moveDelay, ms));
            }
        }
    }

    class MotorInstrument implements Instrument {
        /**
         * The ratio from produced pitch to motor speed.
         *
         * For example, a motor that produces 500Hz at 0.5 should have this as 1000.
         */
        private double pitchRatio;

        // ---

        private DcMotor actuator;
        private boolean isReversed = false;

        MotorInstrument(String deviceName,
                        double pitchRatio) {
            actuator = hardwareMap.dcMotor.get(deviceName);
            this.pitchRatio = pitchRatio;

            // Initialize motor as instrument
            actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        @Override
        public void play(double pitch, int ms) {
            // The current methodology might introduce wear & tear on the brakes.
            // Use responsibly.
            actuator.setPower(pitch / pitchRatio * (isReversed ? -1 : 1));
            sleep(ms);
            actuator.setPower(0);

            // No need to reverse if at rest
            if (pitch != 0.0)
                isReversed = !isReversed;
        }
    }

    // The musical note

    class Note {
        double pitch;
        int beats;

        public Note(double pitch, int beats) {
            this.pitch = pitch;
            this.beats = beats;
        }

        public Note(int beats) {
            this(0, beats);
        }
    }

    // The thread routine

    class TrackExecutor implements Runnable {
        private Instrument instrument;
        private Note[] track;

        TrackExecutor(Instrument instrument, Note[] track) {
            this.instrument = instrument;
            this.track = track;
        }

        @Override
        public void run() {
            for (Note note : track) {
                instrument.play(note.pitch, 60000 / BPM * note.beats);

                if (isStopRequested() || Thread.currentThread().isInterrupted()) {
                    return;
                }
            }
        }
    }
}
