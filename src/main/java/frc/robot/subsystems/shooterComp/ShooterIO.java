package frc.robot.subsystems.shooterComp;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double velocityRadSec;
        public double desiredVelocityRadSec;
        public boolean reachedSetpoint;
        public boolean inShooter;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVelocityRadSec(double goalVelocityRadPerSec, boolean isVar, double Bot, double Top) {}

    public default double getDesiredVelocity() {return 0.0;}

    public default void configureController(SimpleMotorFeedforward vff, PIDController vfb) {}

    public default void setVoltage(double voltage) {}
}
