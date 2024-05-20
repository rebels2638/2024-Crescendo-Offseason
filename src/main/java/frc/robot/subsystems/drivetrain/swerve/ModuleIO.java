package frc.robot.subsystems.drivetrain.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double driveVelocityMps = 0;
        public double drivePositionsMeters = 0;
        public double anglePoseRad = 0;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}
    public default void setDriveVoltage(double voltage) {}
    public default void setAngleVoltage(double voltage) {}
    public default void resetDriveMotors() {}
}
