package frc.robot.subsystems.drivetrain.swerve.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double timestamp;
        public double drivePositionRad;
        public double driveVelocityRadPerSec;
        public double driveCurrentAmps;
        public double driveAppliedVolts;
        public double driveTemperature;

        public Rotation2d steerAbsolutePosition;

        public double steerAbsolutePositionRadians;
        public Rotation2d steerPosition;
        public double steerPositionRad;
        public double steerVelocityRadPerSec;
        public double steerCurrentDrawAmps;
        public double steerAppliedVolts;
        public double steerTemperature;

        public double desiredSteerPositionRad;
        public double desiredDriveVelocityMetersPerSec;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}
    public default void setState(SwerveModuleState state) {}
    public default void setDriveVoltage(double baseUnitMagnitude) {}
    
}
