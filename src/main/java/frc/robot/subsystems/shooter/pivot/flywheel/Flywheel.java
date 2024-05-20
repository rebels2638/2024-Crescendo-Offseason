package frc.robot.subsystems.shooter.pivot.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final FlywheeIOInputsAutoLogged inputs = new FlywheeIOInputsAutoLogged();
    private FlywheeIO io;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);
    
    private double desiredRPM = 0;
    public Flywheel(FlywheeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        double voltage = feedforward.calculate(desiredRPM, Math.signum(desiredRPM));

        if (Math.max(inputs.tAmps, inputs.bAmps) > 40 || 
            Math.max(inputs.tVolts, inputs.tVolts) > 12 || 
            Math.max(inputs.tTemp, inputs.tTemp) >= 94) {
            voltage = 0;
            
            System.err.println("SHOOTER MOTOR TO HOT!");
            Logger.recordOutput("Flywheel/withinOperationRange", false);
        }
        else {
            Logger.recordOutput("Flywheel/withinOperationRange", true);
        }

        io.setVoltage(voltage);
        Logger.recordOutput("Flywheel/calculatedVoltage", voltage);

    }

    public void setRMP(double rpm) {
        desiredRPM = rpm;
    }
}
