package frc.robot.subsystems.shooter.flywheel;
// package frc.robot.subsystems.shooter.pivot.flywheel;

// import com.revrobotics.CANSparkBase.IdleMode;

// import frc.robot.subsystems.indexer.Indexer;

// import com.revrobotics.CANSparkMax;

// public class FlywheeIOSparkMAX implements FlywheeIO{
    
//     private static final double kMOTOR_TO_OUTPUT_RATIO = 2;

//     private CANSparkMax m_motorT = new CANSparkMax(88, CANSparkMax.MotorType.kBrushless);
//     private CANSparkMax m_motorB = new CANSparkMax(89, CANSparkMax.MotorType.kBrushless);   
    
//     public FlywheeIOSparkMAX() {
//         m_motorT.setIdleMode(IdleMode.kBrake);
//         m_motorB.setIdleMode(IdleMode.kBrake);

//         m_motorT.setInverted(false);
//         m_motorB.setInverted(false);
//     }

//     @Override
//     public void updateInputs(FlywheeIOInputs inputs) {
//         inputs.RPM = m_motorT.getEncoder().getVelocity() * kMOTOR_TO_OUTPUT_RATIO;

//         inputs.tAmps = m_motorT.getOutputCurrent();
//         inputs.bAmps = m_motorB.getOutputCurrent();

//         inputs.tTemp = m_motorT.getMotorTemperature();
//         inputs.bTemp = m_motorB.getMotorTemperature();

//         inputs.tVolts = m_motorT.getAppliedOutput() * 12;
//         inputs.bVolts = m_motorB.getAppliedOutput() * 12;
//     }

//     public void setVoltage(double voltage) {
//         m_motorT.setVoltage(voltage);
//         m_motorB.setVoltage(voltage);
//     }
// }
