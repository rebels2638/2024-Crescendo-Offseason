package frc.robot.subsystems.intakeComp;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double velocityRadSec = 0;
        public double velocityMps = 0;
        public double distanceMeters = 0;
        public boolean inIntake = false; 
        public boolean reachedSetpoint = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setVelocityRadSec(double goalVelocityRadPerSec) {}

    public default void configureController(SimpleMotorFeedforward vff, PIDController vfb) {}

    public default void setVoltage(double voltage) {}
    
}
