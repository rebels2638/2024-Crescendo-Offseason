package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.pivot.flywheel.Flywheel;

public class ShooterWindup extends Command {
    private Flywheel flywheelSubsystem;
    public ShooterWindup(Flywheel flywheel) {
        flywheelSubsystem = flywheel;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheelSubsystem.setRPM(170);
    }

    @Override
    public boolean isFinished() {
        return flywheelSubsystem.reachedSetpoint();
    }

    @Override
    public void end(boolean interrupted) {}
}
