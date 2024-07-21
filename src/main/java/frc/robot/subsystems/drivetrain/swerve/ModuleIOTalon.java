package frc.robot.subsystems.drivetrain.swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.lib.util.RebelUtil;

public class ModuleIOTalon implements ModuleIO {

    private TalonFX m_angle;
    private TalonFX m_drive;
    
    private final PIDController m_angleFeedbackController = new PIDController(6, 0.0, 0.01);
    private final PIDController m_driveFeedbackController = new PIDController(0.00, 0, 0);

    private final SimpleMotorFeedforward m_angleFeedforward = new SimpleMotorFeedforward(.23, 0.0000); // .23
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.22, 1.08,  0.0);
    
    private double m_angleVoltage = 0;
    private double m_driveVoltage = 0;
    
    private double anglePositionRad = 0;
    private double drivePositionsMeters = 0;

    private double driveVelocityMps = 0;

    private CANcoder angleEncoder;
    private boolean syncQueued = false;

    public ModuleIOTalon(int id) {
        CANcoderConfiguration endocerConfig = new CANcoderConfiguration();
        endocerConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        endocerConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        switch (id) {
            case 0:
                angleEncoder = new CANcoder(10, "drivetrain");
                endocerConfig.MagnetSensor.MagnetOffset = Constants.DrivetrainConstants.kFRONT_LEFT_ANGLE_OFFSET_DEG / 360;

                m_angle = new TalonFX(5, "drivetrain");
                m_drive = new TalonFX(4, "drivetrain");

                m_angle.setInverted(true);
                m_drive.setInverted(false);
                break;
            case 1:
                angleEncoder = new CANcoder(9, "drivetrain");
                endocerConfig.MagnetSensor.MagnetOffset = Constants.DrivetrainConstants.kFRONT_RIGHT_ANGLE_OFFSET_DEG / 360;

                m_angle = new TalonFX(3, "drivetrain");
                m_drive = new TalonFX(2, "drivetrain");

                m_angle.setInverted(true);
                m_drive.setInverted(false);
                break;
            case 2:
                angleEncoder = new CANcoder(11, "drivetrain");
                endocerConfig.MagnetSensor.MagnetOffset = Constants.DrivetrainConstants.kBACK_LEFT_ANGLE_OFFSET_DEG / 360;

                m_angle = new TalonFX(7, "drivetrain");
                m_drive = new TalonFX(6, "drivetrain");

                m_angle.setInverted(true);
                m_drive.setInverted(false);
                break;

            case 3:
                angleEncoder = new CANcoder(8, "drivetrain");
                endocerConfig.MagnetSensor.MagnetOffset = Constants.DrivetrainConstants.kBACK_RIGHT_ANGLE_OFFSET_DEG / 360;

                m_angle = new TalonFX(0, "drivetrain");
                m_drive = new TalonFX(1, "drivetrain");

                m_angle.setInverted(true);
                m_drive.setInverted(false);
                break;
        }
        
        endocerConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        angleEncoder.getConfigurator().apply(endocerConfig);
        
        m_angle.setNeutralMode(NeutralModeValue.Brake);
        m_drive.setNeutralMode(NeutralModeValue.Brake);

        

        m_angleFeedbackController.setTolerance(Math.toRadians(2));
        m_driveFeedbackController.setTolerance(0.01);

        m_angleFeedbackController.enableContinuousInput(0, 2 * Math.PI);
    }

    @Override
    public void queueSynchronizeEncoders() {
        if (angleEncoder != null) {syncQueued = true;}
    }
    
    @Override 
    public void updateInputs(ModuleIOInputs inputs) {

        inputs.angleVelocityRadPerSec = m_angle.getVelocity().getValueAsDouble() * 2 * Math.PI;
        inputs.driveVelocityMps = m_drive.getVelocity().getValueAsDouble() * 2 * Math.PI * Constants.DrivetrainConstants.kDRIVE_WHEEL_RADIUS_METERS * Constants.DrivetrainConstants.kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO;

        driveVelocityMps = inputs.driveVelocityMps;
        
        anglePositionRad = m_angle.getPosition().getValueAsDouble() * 2 * Math.PI * Constants.DrivetrainConstants.kANGLE_MOTOR_TO_OUTPUT_SHAFT_RATIO;
        drivePositionsMeters = m_drive.getPosition().getValueAsDouble() * 2 * Math.PI * Constants.DrivetrainConstants.kDRIVE_WHEEL_RADIUS_METERS * Constants.DrivetrainConstants.kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO;

        inputs.absEncoderRad = (angleEncoder.getPosition().getValueAsDouble() * Math.PI * 2);

        anglePositionRad = inputs.absEncoderRad;


        inputs.anglePositionRad = anglePositionRad;
        inputs.drivePositionMeters = drivePositionsMeters;

        inputs.angleCurrentAmps = m_angle.getSupplyCurrent().getValueAsDouble();
        inputs.driveCurrentAmps = m_drive.getSupplyCurrent().getValueAsDouble();

        inputs.angleTempC = m_angle.getDeviceTemp().getValueAsDouble();
        inputs.driveTempC = m_drive.getDeviceTemp().getValueAsDouble();

        inputs.angleVoltage = m_angleVoltage;
        inputs.driveVoltage = m_driveVoltage;
    }

    @Override
    public void setState(SwerveModuleState state) {
        if (DriverStation.isTest()) {return;}

        double speed = state.speedMetersPerSecond;
        m_driveVoltage = m_driveFeedbackController.calculate(driveVelocityMps, speed);
        m_driveVoltage += m_driveFeedforward.calculate(speed, Math.signum(m_driveVoltage));

        m_driveVoltage = RebelUtil.constrain(
                         m_driveVoltage, -Constants.DrivetrainConstants.kMAX_DRIVE_VOLTAGE,
                         Constants.DrivetrainConstants.kMAX_DRIVE_VOLTAGE);

        m_drive.setVoltage(m_driveVoltage);

        if (angleEncoder != null && angleEncoder.getMagnetHealth().getValue() == MagnetHealthValue.Magnet_Green && syncQueued) {
            m_angle.setPosition(angleEncoder.getAbsolutePosition().getValueAsDouble());
            syncQueued = false;
        }

        double angle = state.angle.getRadians();

        m_angleVoltage = m_angleFeedbackController.calculate(anglePositionRad, angle);
        m_angleVoltage += m_angleFeedforward.calculate(angle - anglePositionRad, Math.signum(m_angleVoltage)); // TODO: this is scuffed
                         

        m_angleVoltage = RebelUtil.constrain(
                         m_angleVoltage, -Constants.DrivetrainConstants.kMAX_ANGLE_VOLTAGE,
                         Constants.DrivetrainConstants.kMAX_ANGLE_VOLTAGE);

        m_angle.setVoltage(m_angleVoltage);
    }

    @Override
    public void resetPosition() {
        drivePositionsMeters = 0;
    }
}