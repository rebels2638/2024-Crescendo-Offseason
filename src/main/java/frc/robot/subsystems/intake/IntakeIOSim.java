package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;

public class IntakeIOSim implements IntakeIO {
    private static final double kMOTOR_TO_OUTPUT_RATIO = 2;

    private DCMotor m_gearbox = DCMotor.getNeo550(1);
    private FlywheelSim m_motor = new FlywheelSim(m_gearbox, kMOTOR_TO_OUTPUT_RATIO, 0.007);

    private double voltage = 0;
    
    private final Indexer indexer;
    public IntakeIOSim(Indexer indexer) {
        this.indexer = indexer;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs)  {
        m_motor.update(0.020);

        inputs.intakeVelocityMps = m_motor.getAngularVelocityRPM() * 
                                    kMOTOR_TO_OUTPUT_RATIO * 
                                    Math.PI * 2 * 
                                    Constants.IntakeConstants.kROLLER_RADIUS_METERS;

        inputs.intakePoseMeters += inputs.intakeVelocityMps * 0.020;

        inputs.amps = m_motor.getCurrentDrawAmps();

        inputs.temp = 0;

        inputs.volts = voltage;

        inputs.inIntake = indexer.inIntake();
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
        m_motor.setInputVoltage(voltage);
    }
}
