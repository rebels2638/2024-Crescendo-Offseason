package frc.robot.commands.shooterComp;

import edu.wpi.first.wpilibj2.command.Command; // Base class for commands in WPILib.
import frc.robot.subsystems.shooterComp.Shooter; // This is the shooter subsystem.

public class ShooterHold extends Command {

    private final Shooter shooterSubsystem = Shooter.getInstance(); // Get the shooter subsystem.

    public ShooterHold() {
        // Constructor with no parameters.
    }

    // This runs when the command is initialized.
    @Override
    public void initialize() {
        // Set the shooter's velocity to 6.5 rad/sec.
        shooterSubsystem.setVelocityRadSec(6.5, false, 0, 0); // Set velocity for holding.
    }

    // This checks if the command is finished.
    @Override
    public boolean isFinished() {
        return true; // The command completes immediately.
    }
}
