package frc.robot.subsystems.shooter.pivot.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheeIO {
    @AutoLog
    public static class FlywheeIOInputs {
        public double RPM;
        
        public double tTemp;
        public double bTemp;

        public double tAmps;
        public double bAmps;

        public double tVolts;
        public double bVolts;
    }

    public default void updateInputs(FlywheeIOInputs inputs) {}
    public default void setVoltage(double voltage) {}
}
