package frc.robot.subsystems.intakeComp;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;

public class IntakeIONeo extends SubsystemBase implements IntakeIO {
    private final double kMotorToOutputShaftRatio = 0.25;
    private CANSparkMax m_motor = new CANSparkMax(16, CANSparkMax.MotorType.kBrushless); 

    private PIDController velocityFeedBackController = new PIDController(0, 0, 0);
    private SimpleMotorFeedforward velocityFeedForwardController = new SimpleMotorFeedforward(0, 0, 0);

    // private Rev2mDistanceSensor distanceSensor;
    private double currentVelocityRadPerSec;


    private GenericEntry IntakeStatus;

    private final Indexer indexer;

    private static final double kMAX_VOLTAGE = 12;

    public IntakeIONeo(Indexer indexer) {
        this.indexer = indexer;
        m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor.clearFaults(); //TODO: ALWAYS CHECK FOR FAULTS IN COMPETITION DO NOT IGNORE THEM
        m_motor.setInverted(false);
        // distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP, Rev2mDistanceSensor.Unit.kMillimeters, Rev2mDistanceSensor.RangeProfile.kDefault);
        // distanceTolerance = 0.57; //Approximate distance assuming some tolerance, CHECK AGAIN
        // distanceSensor.setEnabled(true);

        //distanceSensor.setAutomaticMode(true); << Probably not required but keep note that we need this if we have several of these 2m dist devices

        IntakeStatus = Shuffleboard.getTab("auto").add("INTAKE STATUS", inIntake()).getEntry();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRadSec = m_motor.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio; // we divide by 60 because the motor out is in RPM
        inputs.velocityMps = m_motor.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio * Math.PI * 2 * Constants.IntakeConstants.kROLLER_RADIUS_METERS; // we divide by 60 because the motor out is in RPM
        inputs.distanceMeters += inputs.velocityMps * 0.020;
        
        inputs.reachedSetpoint = velocityFeedBackController.atSetpoint();
        inputs.inIntake = inIntake();
        currentVelocityRadPerSec = inputs.velocityRadSec;
    }

    @Override
    // should be called periodically
    public void setVelocityRadSec(double goalVelocityRadPerSec) {
         if(!inIntake() && goalVelocityRadPerSec == 0){
             goalVelocityRadPerSec = 2;
        }

        double accel = 0;

        if (goalVelocityRadPerSec > currentVelocityRadPerSec) {
            accel = 1;
        }
        else if (goalVelocityRadPerSec < currentVelocityRadPerSec) {
            accel = -1;
        }



        double feedForwardVoltage = velocityFeedForwardController.calculate(goalVelocityRadPerSec, accel);
        
        velocityFeedBackController.setSetpoint(goalVelocityRadPerSec);
        double feedBackControllerVoltage = velocityFeedBackController.calculate(currentVelocityRadPerSec);

        double outVoltage = feedForwardVoltage + feedBackControllerVoltage;

            
        if (outVoltage > kMAX_VOLTAGE) {
            outVoltage = 12;
        }
        else if (outVoltage < -kMAX_VOLTAGE) {
            outVoltage = -12;
        }
        Logger.recordOutput("Intake/voltageOut", outVoltage);

        // if (goalVelocityRadPerSec == 0) {
        //     m_motor.setVoltage(0);

        // }
        // else {
        //     m_motor.setVoltage(4);
        // }

        m_motor.setVoltage(outVoltage);
        IntakeStatus.setBoolean(inIntake());
    } 

    @Override
    public void setVoltage(double voltage){
        m_motor.setVoltage(voltage);
    }

    @Override
    public void configureController(SimpleMotorFeedforward vff, PIDController vfb ) {
        velocityFeedBackController = vfb;
        velocityFeedForwardController = vff;
    }
    
    public boolean inIntake() {
        //Valid range(?) check aka 2m or less


        /* //Uncomment this whenever you the 2mDistance sensor is there
        if(distanceSensor.isRangeValid()){
            //distanceSensor.setMeasurementPeriod();
            //Using default measurementperiod, we get its range at that moment.
            if(distanceSensor.getRange(Rev2mDistanceSensor.Unit.kMillimeters) < distanceTolerance){
                return true;
            }
            return false;
        }
        else{
            // System.out.println("Out of range");
            return false;
        }
        */

        return indexer.inIntake();

    }

}