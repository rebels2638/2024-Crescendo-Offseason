package frc.robot.subsystems.drivetrain.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double driveVelocityMps = 0;
        public double drivePositionMeters = 0;

        public double angleVelocityRadPerSec = 0;
        public double anglePositionRad = 0;

        public double driveCurrentAmps = 0;
        public double angleCurrentAmps = 0;

        public double driveTempC = 0;
        public double angleTempC = 0;

        public double driveVoltage = 0;
        public double angleVoltage = 0;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}
    public default void setState(SwerveModuleState state, double nextAngle) {}
    public default void resetPosition() {}
}
