package frc.robot.commands.intake;

import frc.robot.lib.input.XboxController; // Library for Xbox controller input.
import edu.wpi.first.wpilibj2.command.Command; // Base class for commands in wpilib.
import frc.robot.subsystems.intakeComp.Intake; // Reference to the intake subsystem.
import frc.robot.subsystems.pivotComp.Pivot; // Reference to the pivot subsystem.

public class IntakeToggle extends Command {

    private int tapped; // Counter for button taps.
    private final Intake intakeSubsystem = Intake.getInstance(); // Singleton instance of the Intake subsystem.
    private final Pivot pivotSubsystem = Pivot.getInstance(); // Singleton instance of the Pivot subsystem.
    private XboxController m_controller; // Reference to the Xbox controller.

    // Constructor that initializes the controller.
    public IntakeToggle(XboxController controller) {
        this.m_controller = controller; // Set the controller for input.
        this.tapped = 0; // Initialize tapped counter.
    }

    // Executes the command logic.
    @Override
    public void execute() {
        // Check if the Y button is pressed on the controller.
        if (this.m_controller.getYButton().getAsBoolean()) {
            this.tapped++; // Increment the tapped counter.

            // Handle different cases based on the number of taps.
            switch (tapped) {
                case 1: // Roll intake
                    // Set intake speed based on pivot angle.
                    if (pivotSubsystem.getDegAngle() < 45) {
                        intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 20)); // Fast intake speed.
                    } else {
                        intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 4)); // Slower intake speed.
                    }
                    break;

                case 2: // Stop intake
                    intakeSubsystem.setVelocityRadSec(0); // Stop the intake.
                    break;

                case 3: // Reverse intake
                    intakeSubsystem.setVelocityRadSec(-1); // Reverse intake.
                    this.tapped = 0; // Reset tapped counter after reversing.
                    break;
            }
        }
    }

    // Determines if the command should end.
    @Override
    public boolean isFinished() {
        return true; // This command finishes immediately after execution.
    }
}
