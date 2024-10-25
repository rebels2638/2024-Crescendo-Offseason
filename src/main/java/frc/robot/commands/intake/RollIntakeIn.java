package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command; // Base class for commands in WPILib.
import frc.robot.subsystems.intakeComp.Intake; // This is the intake system.
import frc.robot.subsystems.pivotComp.Pivot; // This is the pivot system.

public class RollIntakeIn extends Command {

    private final Intake intakeSubsystem = Intake.getInstance(); // Get the intake system.
    private final Pivot pivotSubsystem = Pivot.getInstance(); // Get the pivot system.

    // This constructor sets up the command.
    public RollIntakeIn() {
    }

    // This runs when the command starts.
    @Override
    public void initialize() {
        // Check the angle of the pivot.
        if (pivotSubsystem.getDegAngle() < 45) {
            // If the angle is less than 45 degrees, spin the intake fast.
            intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 10)); // Fast speed for intake.
        } else {
            // If the angle is 45 degrees or more, spin the intake at a slower speed.
            intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 8)); // Slower speed for intake.
        }
        // intakeSubsystem.setVelocityRadSec(0); //Use radians directly

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
