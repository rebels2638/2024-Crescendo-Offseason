package frc.robot.subsystems.shooterComp;

import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import com.revrobotics.CANSparkMaxLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.util.RebelUtil;
import frc.robot.subsystems.indexer.Indexer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


// import com.revrobotics.jni.VL53L0XJNI;

public class ShooterIONeo extends SubsystemBase implements ShooterIO {
    private static final double kMotorToOutputShaftRatio = 1; //Last Checked 2/6/2024
    private double wheelVelo = 0.0;
    private CANSparkMax m_motor1 = new CANSparkMax(13, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax m_motor2 = new CANSparkMax(14, CANSparkMax.MotorType.kBrushless);
    // private CANSparkMax m_motor1 = new CANSparkMax(29, MotorType.kBrushless);
    // private CANSparkMax m_motor2 = new CANSparkMax(23, MotorType.kBrushless);

    private double goalVelocityRadPerSec = 0.0;

    private PIDController velocityFeedBackController = new PIDController(0, 0, 0);
    private SimpleMotorFeedforward velocityFeedForwardController = new SimpleMotorFeedforward(0, 0, 0);

    private double distanceTolerance;
    private double currentVelocityRadPerSec = 0 ;
    private double currentVelocityRadPerSec2 = 0; //Second motor, m_motor2

    private GenericEntry ShooterStatus;
    private GenericEntry InShooter;


    private static final double kMAX_VOLTAGE = 12;
    

    private final Indexer indexer;

    public ShooterIONeo(Indexer indexer) {
        this.indexer = indexer;
        m_motor1.clearFaults();
        m_motor2.clearFaults();

        m_motor1.setIdleMode(IdleMode.kBrake);
        m_motor2.setIdleMode(IdleMode.kBrake);

        m_motor1.setInverted(false);
        m_motor2.setInverted(false); 

        ShooterStatus = Shuffleboard.getTab("auto").add("SHOOTER STATUS", currentVelocityRadPerSec > 60).getEntry();
        InShooter = Shuffleboard.getTab("auto").add("In Shooter", isInShooter()).getEntry();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.velocityRadSec = m_motor1.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio; // we divide by 60 because the motor out is in RPM
        currentVelocityRadPerSec2 = m_motor2.getEncoder().getVelocity() /60 * kMotorToOutputShaftRatio;
        currentVelocityRadPerSec = inputs.velocityRadSec;
        inputs.reachedSetpoint = velocityFeedBackController.atSetpoint();
        inputs.inShooter = isInShooter();
    }

    @Override
    public double getDesiredVelocity() {
        return goalVelocityRadPerSec;
    }

    @Override
    // should be called periodically
    public void setVelocityRadSec(double goalVelocityRadPerSec, boolean isVariable, double BottomSpeed, double TopSpeed) {
        this.goalVelocityRadPerSec = goalVelocityRadPerSec;
        if(isVariable){
            double ffVelo = 0;
        if (BottomSpeed > currentVelocityRadPerSec) {
            ffVelo = 1; 
        }
        else if (BottomSpeed < currentVelocityRadPerSec) {
            ffVelo = -1; 
        }
            
            double feedForwardVoltage = velocityFeedForwardController.calculate(BottomSpeed, ffVelo);
            double outVoltage1 = feedForwardVoltage;
                ffVelo = 0;
        if (TopSpeed > currentVelocityRadPerSec) {
            ffVelo = 1; 
        }
        else if (TopSpeed < currentVelocityRadPerSec) {
            ffVelo = -1; 
        }
            feedForwardVoltage = velocityFeedForwardController.calculate(TopSpeed, ffVelo);
            double outVoltage2 = feedForwardVoltage; 

            m_motor1.setVoltage(outVoltage2);
            m_motor2.setVoltage(outVoltage1);
        }else{
        double ffVelo = 0;
        if (goalVelocityRadPerSec > currentVelocityRadPerSec) {
            ffVelo = 1; 
        }
        else if (goalVelocityRadPerSec < currentVelocityRadPerSec) {
            ffVelo = -1; 
        }

        double feedForwardVoltage = velocityFeedForwardController.calculate(goalVelocityRadPerSec, ffVelo);
        velocityFeedBackController.setSetpoint(goalVelocityRadPerSec);
        double feedBackControllerVoltage = velocityFeedBackController.calculate(currentVelocityRadPerSec);
        double outVoltage = feedForwardVoltage + feedBackControllerVoltage;
        // double outVoltage = feedBackControllerVoltage;
            
        outVoltage = RebelUtil.constrain(outVoltage, -12, 12);
        // if (goalVelocityRadPerSec > 0) {
        //      outVoltage = 6;
        // }
        // else {
        //     outVoltage = 0;
        // }
        Logger.recordOutput("Shooter/voltageOut", outVoltage);
        m_motor1.setVoltage(outVoltage);
        m_motor2.setVoltage(outVoltage);

        ShooterStatus.setBoolean(currentVelocityRadPerSec > 60);
        }

        InShooter.setBoolean(isInShooter());
        this.goalVelocityRadPerSec = goalVelocityRadPerSec;
    } 

    @Override
    public void setVoltage(double voltage){
        m_motor1.setVoltage(voltage);
    }

    @Override
    public void configureController(SimpleMotorFeedforward vff, PIDController vfb ) {
        velocityFeedBackController = vfb;
        velocityFeedForwardController = vff;
    }

    private boolean isInShooter(){
        return indexer.inShooter();
    }

}