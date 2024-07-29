package frc.robot.subsystems.drivetrain.vision;

import org.littletonrobotics.junction.AutoLog;

public interface NoteDetectorIO {
    @AutoLog
    public static class NoteDetectorIOInputs {
        public double txRadians = 0;
        public double tyRadians = 0;
        public boolean hasTargets = false;
    }

    public default void updateInputs(NoteDetectorIOInputs inputs) {};
}
