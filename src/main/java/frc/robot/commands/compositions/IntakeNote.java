package frc.robot.commands.compositions;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup; // Group for running commands in parallel.
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup; // Group for executing commands in sequence.
import edu.wpi.first.wpilibj2.command.WaitCommand; // Command that pauses execution for a specified time.

import frc.robot.Constants; // Robot constants, including operation modes.
import frc.robot.commands.drivetrain.DriveToNote; // Command to drive the robot to a detected note.
import frc.robot.commands.intake.InIntake; // Command for bringing objects into the intake.
import frc.robot.commands.intake.OutIntake; // Command for pushing objects out of the intake.
import frc.robot.commands.intake.RollIntakeIn; // Command to roll the intake mechanism in.
import frc.robot.commands.intake.RollIntakeInSlow; // Command to slowly roll the intake mechanism in.
import frc.robot.commands.intake.RollIntakeOut; // Command to roll the intake mechanism out.
import frc.robot.commands.intake.StopIntake; // Command to stop the intake mechanism.
import frc.robot.commands.pivot.PivotToTorus; // Command to pivot to a position relative to a torus.
import frc.robot.commands.pivot.PivotTurtle; // Command to pivot to a turtle position.
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive; // Swerve drive subsystem for movement.
import frc.robot.subsystems.drivetrain.vision.NoteDetector; // Vision system for detecting notes.
import frc.robot.subsystems.intakeComp.Intake; 

public class IntakeNote extends SequentialCommandGroup {

    // Constructor to initialize the IntakeNote command group.
    public IntakeNote(SwerveDrive swerveDrive, Intake intakeSubsystem, NoteDetector noteDetector) {

        // Check if the robot is in simulation mode.
        if (Constants.currentMode == Constants.Mode.SIM) {
            addCommands(
                // Run rolling the intake in and pivoting in parallel.
                new ParallelCommandGroup(
                    new RollIntakeIn(), // Begin rolling the intake in.
                    new PivotToTorus() // Pivot to the torus position.
                ),
                // Drive to the detected note.
                new DriveToNote(swerveDrive, intakeSubsystem, noteDetector),
                new StopIntake(), // Stop the intake mechanism.
                new PivotTurtle() // Pivot to a turtle position.
            );

        } else {
            addCommands(
                // Run rolling the intake in and pivoting in parallel.
                new ParallelCommandGroup(
                    new RollIntakeIn(), // Begin rolling the intake in.
                    new PivotToTorus() // Pivot to the torus position.
                ),
                new WaitCommand(.5),
                // Drive to the detected note.
                new DriveToNote(swerveDrive, intakeSubsystem, noteDetector),
                new StopIntake(), // Stop the intake mechanism.
                new PivotTurtle(), // Pivot to a turtle position.
                new RollIntakeOut(), // Roll the intake out.
                new WaitCommand(0.15), // Wait for 0.15 seconds.
                new OutIntake(), // Push the object out of the intake.
                new StopIntake(), // Stop the intake again.
                new RollIntakeInSlow(), // Slowly roll the intake in.
                new InIntake(), // Bring the object into the intake.
                new WaitCommand(0.1), // Wait for 0.1 seconds.
                new StopIntake() // Finally stop the intake.
            );
        }
    }
}
