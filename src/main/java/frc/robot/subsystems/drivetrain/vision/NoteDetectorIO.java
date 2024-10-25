package frc.robot.subsystems.drivetrain.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Translation2d;

public interface NoteDetectorIO {
    @AutoLog
    public static class NoteDetectorIOInputs {
        public double txRadians = 0;
        public double tyRadians = 0;
        public boolean hasTargets = false;
        public Translation2d bestNote = new Translation2d();
        public double totalLatencySeconds = 0;
    }

    public default void updateInputs(NoteDetectorIOInputs inputs) {};
    public default Translation2d getPose() { return new Translation2d(); };

}
