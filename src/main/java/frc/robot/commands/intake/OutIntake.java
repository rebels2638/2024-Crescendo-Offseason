package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command; // Base class for commands in WPILib.
import frc.robot.subsystems.intakeComp.Intake; // Reference to the intake subsystem.

public class OutIntake extends Command {

    private Intake intakeNeo = Intake.getInstance(); // Singleton instance of the Intake subsystem.
    private boolean isIn = false; // Flag to track whether the object is in the intake.

    // Constructor for OutIntake command.
    public OutIntake() {
    }

    // Executes the command logic.
    @Override
    public void execute() {
        // Update the isIn flag based on the intake status.
        isIn = intakeNeo.inIntake();
        // Debugging statement (commented out).
        // System.out.println("Out intake : " + !isIn);
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean isInterrupted) {
        // No specific action on end; placeholder for future cleanup if necessary.
        return;
    }

    // Returns true when the object has been successfully ejected from the intake.
    @Override
    public boolean isFinished() {
        return !isIn; // Command finishes when the object is no longer in the intake.
    }
}
