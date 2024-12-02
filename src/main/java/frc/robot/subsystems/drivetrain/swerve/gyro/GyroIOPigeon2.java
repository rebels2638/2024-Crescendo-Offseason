package frc.robot.subsystems.drivetrain.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.swerve.Phoenix6Odometry;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 gyro;
    private final StatusSignal<Double> yawSignal;
    private final StatusSignal<Double> angularVelocitySignal;
    private final StatusSignal<Double> rollSignal;
    private final StatusSignal<Double> rollVelocitySignal;
    private final StatusSignal<Double> pitchSignal;
    private final StatusSignal<Double> pitchVelocitySignal;
    private final StatusSignal<Double> accelerationXSignal;
    private final StatusSignal<Double> accelerationYSignal;

    private final Phoenix6Odometry odom;

    public GyroIOPigeon2() {
        odom = Phoenix6Odometry.getInstance();
        gyro = new Pigeon2(0, "canivore");
        Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPose.MountPoseYaw = 0;
        config.MountPose.MountPosePitch = 0;
        config.GyroTrim.GyroScalarZ = 0;
        gyro.getConfigurator().apply(config);

        yawSignal = gyro.getYaw().clone();
        angularVelocitySignal = gyro.getAngularVelocityZDevice().clone();

        rollSignal = gyro.getRoll().clone();
        rollVelocitySignal = gyro.getAngularVelocityXWorld().clone();

        pitchSignal = gyro.getPitch().clone();
        pitchVelocitySignal = gyro.getAngularVelocityYWorld().clone();

        accelerationXSignal = gyro.getAccelerationX().clone();
        accelerationYSignal = gyro.getAccelerationY().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                yawSignal,
                angularVelocitySignal,
                rollSignal,
                rollVelocitySignal,
                pitchSignal,
                pitchVelocitySignal,
                accelerationXSignal,
                accelerationYSignal);

        odom.registerSignal(gyro, yawSignal);
        odom.registerSignal(gyro, angularVelocitySignal);
        odom.registerSignal(gyro, rollSignal);
        odom.registerSignal(gyro, rollVelocitySignal);
        odom.registerSignal(gyro, pitchSignal);
        odom.registerSignal(gyro, pitchVelocitySignal);
        odom.registerSignal(gyro, accelerationXSignal);
        odom.registerSignal(gyro, accelerationYSignal);

        gyro.optimizeBusUtilization();
    }

    @Override
    public synchronized void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                        yawSignal,
                        angularVelocitySignal,
                        rollSignal,
                        rollVelocitySignal,
                        pitchSignal,
                        pitchVelocitySignal,
                        accelerationXSignal,
                        accelerationYSignal).isOK();

        inputs.yaw = Units.degreesToRadians(BaseStatusSignal.getLatencyCompensatedValue(yawSignal, angularVelocitySignal));
        inputs.pitch = Units.degreesToRadians(BaseStatusSignal.getLatencyCompensatedValue(pitchSignal, pitchVelocitySignal));
        inputs.angularVelocity = Units.degreesToRadians(angularVelocitySignal.getValue());
        inputs.angularVelocityDegrees = angularVelocitySignal.getValue();
        inputs.roll = Units.degreesToRadians(BaseStatusSignal.getLatencyCompensatedValue(rollSignal, rollVelocitySignal));
        inputs.accelerationX = accelerationXSignal.getValue();
        inputs.accelerationY = accelerationYSignal.getValue();
    }

    @Override
    public int getCanDeviceId() {
        return 0;
    }
}