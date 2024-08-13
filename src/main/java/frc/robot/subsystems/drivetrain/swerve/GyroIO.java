package frc.robot.subsystems.drivetrain.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public Rotation2d yaw = new Rotation2d();
        public boolean isConnected = false;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
        public double yawVelocityRadPerSec = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {};

}
