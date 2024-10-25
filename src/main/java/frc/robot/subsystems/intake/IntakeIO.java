package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeVelocityMps = 0;
        public double intakePoseMeters = 0;
        public double temp = 0;
        public double volts = 0;
        public double amps = 0;
        public boolean inIntake = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {};
    public default void setVoltage(double voltage) {}

}
