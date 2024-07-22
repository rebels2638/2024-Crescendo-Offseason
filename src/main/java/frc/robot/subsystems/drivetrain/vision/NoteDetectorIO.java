package frc.robot.subsystems.drivetrain.vision;

import org.littletonrobotics.junction.AutoLog;

public interface NoteDetectorIO {
    @AutoLog
    public static class NoteDetectorIOInputs {
        public double vxRadians = 0;
        public double vyRadians = 0;
        public boolean hasTargets = false;
    }

    public default void updateInputs(NoteDetectorIOInputs inputs) {};
}
