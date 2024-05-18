package frc.robot.subsystems.drivetrain.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Module extends SubsystemBase {
    private static final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final ModuleIO io;
    private final int id;

    private SwerveModuleState desiredState;

    private final PIDController driveMotorFeedbackController = new PIDController(0, 0, 0);
    private final PIDController angleMotorFeedbackController = new PIDController(0, 0, 0);

    private final SimpleMotorFeedforward driveMotorFeedforwardController = new SimpleMotorFeedforward(0, 0);
    private final SimpleMotorFeedforward angleMotorFeedforwardController = new SimpleMotorFeedforward(0, 0);

    public Module(ModuleIO io, int id) {
        this.io = io;
        this.id = id;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Module/" + id, inputs);
        
        driveMotorFeedbackController.setSetpoint(desiredState.speedMetersPerSecond);
        double driveFBVoltage = angleMotorFeedbackController.calculate(inputs.driveVelocityMps);
        double driveFFVoltage = 
            driveMotorFeedforwardController.calculate(inputs.driveVelocityMps, Math.signum(desiredState.speedMetersPerSecond));
        
        angleMotorFeedbackController.setSetpoint(desiredState.angle.getRadians());
        double angleFBVoltage = angleMotorFeedbackController.calculate(inputs.anglePoseRad);
        double angleFFVoltage = 
            angleMotorFeedforwardController.calculate(inputs.anglePoseRad, Math.signum(inputs.anglePoseRad));

        io.setAngleVoltage(angleFBVoltage + angleFFVoltage);
        io.setDriveVoltage(driveFBVoltage + driveFFVoltage);
    }
    
    public void setModuleState(SwerveModuleState state) {
        desiredState = state;
    }
    public SwerveModuleState getModuleState() {
        SwerveModuleState state = new SwerveModuleState();
        state.angle = new Rotation2d(inputs.anglePoseRad);
        state.speedMetersPerSecond = inputs.driveVelocityMps;

        return state;
    }
}
