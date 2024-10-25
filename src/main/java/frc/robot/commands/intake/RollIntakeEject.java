package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command; // Base class for commands in WPILib.
import frc.robot.subsystems.intakeComp.Intake; // This is the intake system.

public class RollIntakeEject extends Command {

    private final Intake intakeSubsystem = Intake.getInstance(); // Get the intake system.

    // This constructor sets up the command.
    public RollIntakeEject() {
    }

    // This runs when the command starts.
    @Override
    public void initialize() {
        // Set the intake to spin backwards to push out an object.
        intakeSubsystem.setVelocityRadSec(Math.toRadians(-180)); // Speed for ejecting.
    }

    // This runs when the command ends or is stopped.
    @Override
    public void end(boolean isInterrupted) {
        // Optionally stop the intake when the command ends (currently not used).
        // intakeSubsystem.setVelocityRadSec(0); // Stop spinning.
    }

    // This checks if the command is done.
    @Override
    public boolean isFinished() {
        return true; // The command is done immediately.
    }
}
