package frc.robot.subsystems.drivetrain.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class GyroIONavX implements GyroIO {
    private AHRS gyro;
    
    public GyroIONavX() {
        try {
            gyro = new AHRS(Port.kMXP);
        }
        catch (RuntimeException ex) {
            System.err.println("Error initilizing Gyro");
        }
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = gyro.isConnected();
        if (inputs.isConnected) {
            inputs.yaw = new Rotation2d(-Math.toRadians(gyro.getYaw()));
        }
    }

}
