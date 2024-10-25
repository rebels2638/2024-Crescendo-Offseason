package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public boolean inIntake = false;
        public boolean inShooter = false;
    }

    public default void updateInputs(IndexerIOInputs inputs) {};
}
