package frc.robot.subsystems.drivetrain.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.lib.util.RebelUtil;

public class ModuleIOSim implements ModuleIO {
    private DCMotor m_gearBoxAngle = DCMotor.getFalcon500Foc(1);
    private DCMotor m_gearBoxDrive = DCMotor.getFalcon500Foc(1);

    private FlywheelSim m_angleSim = new FlywheelSim(m_gearBoxAngle, Constants.DrivetrainConstants.kANGLE_MOTOR_TO_OUTPUT_SHAFT_RATIO, 0.007);
    private FlywheelSim m_driveSim = new FlywheelSim(m_gearBoxDrive, Constants.DrivetrainConstants.kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO, 0.007);
    
    private static final PIDController m_angleFeedbackController = new PIDController(1, 0, 0);
    private static final PIDController m_driveFeedbackController = new PIDController(1, 0, 0);

    private static final SimpleMotorFeedforward m_angleFeedforward = new SimpleMotorFeedforward(0, 0);
    private static final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 0);
    
    private double m_angleVoltage = 0;
    private double m_driveVoltage = 0;
    
    private double anglePositionRad = 0;
    private double drivePositionsMeters = 0;

    private double prevTime = Timer.getFPGATimestamp();
    
    @Override 
    public void updateInputs(ModuleIOInputs inputs) {
        m_angleSim.update(0.020);
        m_driveSim.update(0.020);

        inputs.angleVelocityRadPerSec = m_angleSim.getAngularVelocityRPM();
        inputs.driveVelocityMps = m_driveSim.getAngularVelocityRPM();
        
        anglePositionRad += inputs.angleVelocityRadPerSec * (Timer.getFPGATimestamp() - prevTime);
        drivePositionsMeters += inputs.driveVelocityMps * (Timer.getFPGATimestamp() - prevTime);

        inputs.anglePositionRad = anglePositionRad;
        inputs.drivePositionMeters = drivePositionsMeters;

        inputs.angleCurrentAmps = m_angleSim.getCurrentDrawAmps();
        inputs.driveCurrentAmps = m_driveSim.getCurrentDrawAmps();

        inputs.angleTempC = 0;
        inputs.driveTempC = 0;

        inputs.angleVoltage = m_angleVoltage;
        inputs.driveVoltage = m_driveVoltage;

        prevTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setState(SwerveModuleState state) {
        double speed = state.speedMetersPerSecond;
        m_driveVoltage = m_driveFeedforward.calculate(speed, Math.signum(speed)) + 
                         m_driveFeedbackController.calculate(m_driveSim.getAngularVelocityRPM(), speed);

        m_driveVoltage = RebelUtil.constrain(
                         m_driveVoltage, -Constants.DrivetrainConstants.kMAX_DRIVE_VOLTAGE,
                         Constants.DrivetrainConstants.kMAX_DRIVE_VOLTAGE);

        m_driveSim.setInputVoltage(m_driveVoltage);

        double angle = state.angle.getRadians();
        m_angleVoltage = m_angleFeedforward.calculate(angle, Math.signum(angle)) + 
                         m_angleFeedbackController.calculate(anglePositionRad, angle);

        m_angleVoltage = RebelUtil.constrain(
                         m_angleVoltage, -Constants.DrivetrainConstants.kMAX_ANGLE_VOLTAGE,
                         Constants.DrivetrainConstants.kMAX_ANGLE_VOLTAGE);

        m_angleSim.setInputVoltage(m_angleVoltage);
    }

    @Override
    public void resetPosition() {
        drivePositionsMeters = 0;
    }
}
