package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.pivot.flywheel.Flywheel;

public class ShooterStop extends Command {
    private Flywheel flywheelSubsystem;
    public ShooterStop(Flywheel flywheel) {
        flywheelSubsystem = flywheel;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheelSubsystem.setRMP(0);
    }

    @Override
    public boolean isFinished() {
        return flywheelSubsystem.reachedSetpoint();
    }

    
}
