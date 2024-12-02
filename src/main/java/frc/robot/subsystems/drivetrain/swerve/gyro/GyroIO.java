package frc.robot.subsystems.drivetrain.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean connected = true;
        public double yaw = 0.0;
        public double pitch = 0.0;
        public double roll = 0.0;
        public double angularVelocity = 0.0;
        public double angularVelocityDegrees = 0.0;
        public double accelerationX = 0.0;
        public double accelerationY = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default int getCanDeviceId() {
        return 0;
    }
}