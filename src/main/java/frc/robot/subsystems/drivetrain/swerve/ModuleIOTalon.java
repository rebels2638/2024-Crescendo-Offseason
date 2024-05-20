package frc.robot.subsystems.drivetrain.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public class ModuleIOTalon implements ModuleIO{
    private TalonFX m_drive = new TalonFX(21);
    private TalonFX m_angle = new TalonFX(20); 
    private CANcoder angleEncoder;

    public ModuleIOTalon(int id) {
        // basic CANcoder config
        CANcoderConfiguration endocerConfig = new CANcoderConfiguration();
        endocerConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        endocerConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        switch (id) {
            case 0:
                angleEncoder = new CANcoder(0, "drivetrain");
                endocerConfig.MagnetSensor.MagnetOffset = Constants.DrivetrainConstants.frontLeftAngleOffsetDeg;
            case 1:
                angleEncoder = new CANcoder(1, "drivetrain");
                endocerConfig.MagnetSensor.MagnetOffset = Constants.DrivetrainConstants.frontRightAngleOffsetDeg;
            case 2:
                angleEncoder = new CANcoder(2, "drivetrain");
                endocerConfig.MagnetSensor.MagnetOffset = Constants.DrivetrainConstants.backLeftAngleOffsetDeg;
            case 3:
                angleEncoder = new CANcoder(3, "drivetrain");
                endocerConfig.MagnetSensor.MagnetOffset = Constants.DrivetrainConstants.backRightAngleOffsetDeg;

            angleEncoder.getConfigurator().apply(endocerConfig);
            
            //motor configs
            m_drive.setNeutralMode(NeutralModeValue.Brake);
            m_angle.setNeutralMode(NeutralModeValue.Brake);
        }

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // TODO: Check for fauly readings in encoder
        inputs.anglePoseRad = angleEncoder.getPosition().getValueAsDouble() * 2 * Math.PI;

        inputs.driveVelocityMps = 
            m_drive.getVelocity().getValueAsDouble() *
            Constants.DrivetrainConstants.driveMotorOutputToShaftRatio * 
            Constants.DrivetrainConstants.driveWheelRadiusMeters * 2 * Math.PI;
            
        inputs.drivePositionsMeters = 
            m_drive.getPosition().getValueAsDouble() *
            Constants.DrivetrainConstants.driveMotorOutputToShaftRatio * 
            Constants.DrivetrainConstants.driveWheelRadiusMeters * 2 * Math.PI;
    }

    @Override
    public void setDriveVoltage(double voltage) {
        m_drive.setVoltage(voltage);
    }

    @Override
    public void setAngleVoltage(double voltage) {
        m_angle.setVoltage(voltage);
    }


}
