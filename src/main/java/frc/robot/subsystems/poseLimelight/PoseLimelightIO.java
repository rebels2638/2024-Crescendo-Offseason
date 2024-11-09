package frc.robot.subsystems.poseLimelight;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface PoseLimelightIO {
    @AutoLog
    public static class PoseLimelightIOInputs {
        public Pose2d estimatedPose = new Pose2d();
        public double timestampSeconds = 0;
        public boolean hasValidTargets = false;
        public int primaryTagId = 0;
        public double tx = 0;
        public double ty = 0;
    }

    public default void updateInputs(PoseLimelightIOInputs inputs) {}
}