package frc.robot.subsystems.shooter.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double angleRad;
        
        public double tTemp;
        public double bTemp;

        public double tAmps;
        public double bAmps;

        public double tVolts;
        public double bVolts;
    }

    public default void updateInputs(PivotIOInputs inputs) {}
    public default void setVoltage(double voltage) {}
}
