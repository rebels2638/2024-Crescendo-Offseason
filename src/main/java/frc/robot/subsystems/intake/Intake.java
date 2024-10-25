package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.util.RebelUtil;
import frc.robot.subsystems.indexer.Indexer;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final PIDController m_velocityFeedbackController;
    private final SimpleMotorFeedforward m_velocityFeedforwardController;
    
    private double desiredSpeedMps = 0;
    
    public Intake(Indexer indexer) {
        switch (Constants.currentMode) {
            case SIM:
                io = new IntakeIOSim(indexer);
                m_velocityFeedbackController = new PIDController(0.3, 0, 0);
                m_velocityFeedforwardController = new SimpleMotorFeedforward(0, 0.006944444444);
                break;
        
            default:
                io = new IntakeIO() {
                    
                };
                m_velocityFeedbackController = new PIDController(0, 0, 0);
                m_velocityFeedforwardController = new SimpleMotorFeedforward(0, 0);
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        double voltage = m_velocityFeedbackController.calculate(inputs.intakeVelocityMps, desiredSpeedMps) + 
                         m_velocityFeedforwardController.calculate(desiredSpeedMps);
        RebelUtil.constrain(voltage, -12, 12);

        if (inputs.amps> 40 || 
            inputs.volts > 12 || 
            inputs.temp >= 94) {
            voltage = 0;
            
            System.err.println("SHOOTER MOTOR NOT WITHIN OPERATION RANGE");
            Logger.recordOutput("Intake/withinOperationRange", false);
        }
        else {
            Logger.recordOutput("Intake/withinOperationRange", true);
        }

        io.setVoltage(voltage);
        Logger.recordOutput("Intake/calculatedVoltage", voltage);

    }

    public void setSpeedMps(double mps) {
        desiredSpeedMps = mps;
        Logger.recordOutput("Intake/desiredSpeedMps", desiredSpeedMps);
    }

    public boolean reachedSetpoint() {
        return Math.abs(inputs.intakeVelocityMps - desiredSpeedMps) <= 0.02;
    }
    
    public boolean inIntake() {
        return inputs.inIntake;
    }

    public double getPoseMeters() {
        return inputs.intakePoseMeters;
    }

    public double getVelocityMps() {
        return inputs.intakeVelocityMps;
    }
}
