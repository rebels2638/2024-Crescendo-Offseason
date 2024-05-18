package frc.robot.subsystems.drivetrain.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class GyroIONavex implements GyroIO {
    private AHRS gyro;
    private Rotation3d offset = new Rotation3d();
    
    public GyroIONavex() {
        try {
            gyro = new AHRS(Port.kMXP);
        }
        catch (RuntimeException ex) {
            System.err.println("Error initilizing Gyro");
        }
    }

    @Override
    public void setOffset(Rotation3d offset) {
        this.offset = offset;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = gyro.isConnected();
        if (inputs.isConnected) {
            inputs.yaw = new Rotation2d(Math.toRadians(gyro.getYaw()) - offset.getZ());
        }
    }

    @Override
    public void zero() {
        offset = gyro.getRotation3d();
    }

    @Override
    public void reset(Rotation3d inital) {
        offset = gyro.getRotation3d().plus(inital);
    }

}
