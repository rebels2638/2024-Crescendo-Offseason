package frc.robot.commands.shooterComp;

import edu.wpi.first.wpilibj2.command.Command; // Base class for commands in WPILib.
import frc.robot.subsystems.shooterComp.Shooter; // This is the shooter subsystem.

public class InShooter extends Command {

    private boolean isIn = false; // Flag to check if the item is in the shooter.

    // Constructor
    public InShooter() {
    }

    // This runs repeatedly while the command is scheduled.
    @Override
    public void execute() {
        isIn = Shooter.getInstance().inShooter(); // Update the status based on the shooter's state.
    }

    // This runs when the command ends or is interrupted.
    @Override
    public void end(boolean isInterrupted) {
        // No specific action on end for now; can be expanded as needed.
        return;
    }

    // This checks if the command is finished.
    @Override
    public boolean isFinished() {
        return isIn; // Command finishes when the item is in the shooter.
    }
}
