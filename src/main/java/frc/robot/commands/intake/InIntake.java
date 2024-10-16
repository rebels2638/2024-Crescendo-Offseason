package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command; // Base class for commands in wpilib.
import frc.robot.subsystems.intakeComp.Intake; // Reference to the intake subsystem.

public class InIntake extends Command {

    private Intake intakeNeo = Intake.getInstance(); // Singleton instance of the Intake subsystem.
    private boolean isIn = false; // Flag to track if the object is fully in.

    // Constructor for InIntake command.
    public InIntake() {
    }

    // Executes the command logic.
    @Override
    public void execute() {
        // Call the intake method to bring an object in and update the flag.
        isIn = intakeNeo.inIntake();
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean isInterrupted) {
        // If interrupted, set the flag to true to indicate the object is fully in.
        if (isInterrupted) {
            isIn = true;
        }
    }

    // Returns true when the intake operation is complete.
    @Override
    public boolean isFinished() {
        return isIn; // Command finishes when the object is in.
    }
}
