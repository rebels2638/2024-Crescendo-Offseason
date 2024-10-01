package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import com.revrobotics.CANSparkMax;

public class PivotIOSparkMAX implements PivotIO {
    
    private static final double kMOTOR_TO_OUTPUT_RATIO = 2;

    private CANSparkMax m_motorR = new CANSparkMax(88, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax m_motorL = new CANSparkMax(89, CANSparkMax.MotorType.kBrushless);   
    
    private final ProfiledPIDController feedbackController = 
        new ProfiledPIDController(1, 0, 0, new Constraints(2 * Math.PI * .25, 2));
    private final ArmFeedforward feedforwardController = new ArmFeedforward(0, 0, 0);

    private double measuredAngleRad = 0; 
    
    public PivotIOSparkMAX() {
        m_motorR.setIdleMode(IdleMode.kBrake);
        m_motorL.setIdleMode(IdleMode.kBrake);

        m_motorR.setInverted(false);
        m_motorL.setInverted(false);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.angleRad = m_motorR.getEncoder().getPosition() * kMOTOR_TO_OUTPUT_RATIO * Math.PI * 2;
        measuredAngleRad = inputs.angleRad;

        inputs.tAmps = m_motorR.getOutputCurrent();
        inputs.bAmps = m_motorL.getOutputCurrent();

        inputs.tTemp = m_motorR.getMotorTemperature();
        inputs.bTemp = m_motorL.getMotorTemperature();

        inputs.tVolts = m_motorR.getAppliedOutput() * 12;
        inputs.bVolts = m_motorL.getAppliedOutput() * 12;
    }

    public void setAngleRad(double desiredAngleRad) {
        double voltage = feedbackController.calculate(measuredAngleRad, desiredAngleRad);
    }
}
