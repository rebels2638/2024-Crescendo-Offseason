package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup; // Group to execute commands in sequence.
import edu.wpi.first.wpilibj2.command.WaitCommand; // Command to add a pause in the sequence.

import frc.robot.commands.intake.InIntake; // Command to bring objects into the intake.
import frc.robot.commands.intake.OutIntake; // Command to push objects out of the intake.
import frc.robot.commands.intake.RollIntakeIn; // Command to roll the intake mechanism in.
import frc.robot.commands.intake.RollIntakeOut; // Command to roll the intake mechanism out.
import frc.robot.commands.intake.StopIntake; // Command to stop the intake.
import frc.robot.commands.pivot.PivotMidway; // Command to pivot to a midway position.
import frc.robot.commands.pivot.PivotToTorus; // Command to pivot to the torus position.
import frc.robot.commands.pivot.PivotTurtle; // Command to pivot to a turtle position.
import frc.robot.commands.intake.RollIntakeInSlow; // Command to slowly roll the intake in.

public class IntakeNoteAuto extends SequentialCommandGroup {

    // Constructor to initialize the IntakeNoteAuto command group.
    public IntakeNoteAuto() {
        addCommands(
            new StopIntake(), // Ensure intake is stopped before starting.
            new PivotToTorus(), // Pivot to the torus position.
            new RollIntakeIn(), // Begin rolling the intake mechanism in.
            new InIntake(), // Activate intake to bring an object in.
            new StopIntake(), // Stop the intake after bringing the object in.
            new PivotMidway(), // Pivot to a midway position.
            new RollIntakeOut(), // Roll the intake out.
            new OutIntake(), // Push the object out of the intake.
            new StopIntake(), // Stop the intake after pushing out.
            new RollIntakeInSlow(), // Slowly roll the intake in.
            new WaitCommand(0.1), // Wait for 0.1 seconds.
            new StopIntake(), // Stop the intake.
            new PivotTurtle() // Pivot to the turtle position.
        );
    }
}
