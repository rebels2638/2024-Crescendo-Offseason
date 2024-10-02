package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.InstantCommand; // Command that runs once immediately.
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup; // Group for running commands in sequence.

import frc.robot.commands.drivetrain.StopDrive; // Command to stop the drive subsystem.
import frc.robot.commands.intake.StopIntake; // Command to stop the intake subsystem.
import frc.robot.commands.pivot.PivotTurtle; // Command to adjust the pivot to a predefined position.
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive; // Swerve drive subsystem.

public class CancelIntakeNote extends SequentialCommandGroup {

    // Constructor to initialize the CancelIntakeNote command group.
    public CancelIntakeNote(SequentialCommandGroup c, SequentialCommandGroup c2, SwerveDrive swerveDrive) {

        // Check if both command groups are not null.
        if(c != null && c2 != null) {
            addCommands(
                new StopDrive(swerveDrive), // Stop the drive subsystem.
                new InstantCommand(()->c.cancel()), // Cancel the first command group.
                new InstantCommand(()->c2.cancel()), // Cancel the second command group.
                new StopIntake(), // Stop the intake subsystem.
                new PivotTurtle() // Pivot the robot to a turtle position.
            );
        }

        // If only the first command group is not null.
        else if(c != null) {
            addCommands(
                new StopDrive(swerveDrive), // Stop the drive subsystem.
                new InstantCommand(()->c.cancel()), // Cancel the first command group.
                new StopIntake(), // Stop the intake subsystem.
                new PivotTurtle() // Pivot the robot to a turtle position.
            );
        }

        // If only the second command group is not null.
        else if(c2 != null) {
            addCommands(
                new StopDrive(swerveDrive), // Stop the drive subsystem.
                new InstantCommand(()->c2.cancel()), // Cancel the second command group.
                new StopIntake(), // Stop the intake subsystem.
                new PivotTurtle() // Pivot the robot to a turtle position.
            );
        }

        // If both command groups are null, just stop drive and intake and pivot.
        else {
            addCommands(
                new StopDrive(swerveDrive), // Stop the drive subsystem.
                new StopIntake(), // Stop the intake subsystem.
                new PivotTurtle() // Pivot the robot to a turtle position.
            );
        }
    }
}
