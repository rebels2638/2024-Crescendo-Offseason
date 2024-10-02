package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup; // Group for running commands in parallel.
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup; // Group for running commands in sequence.
import edu.wpi.first.wpilibj2.command.WaitCommand; // Command that pauses for a specified time.

import frc.robot.commands.intake.OutIntake; // Command to push objects out of the intake.
import frc.robot.commands.intake.RollIntakeInSlow; // Command to slowly roll the intake in.
import frc.robot.commands.intake.StopIntake; // Command to stop the intake.
import frc.robot.commands.shooterComp.InShooter; // Command to shoot objects into the shooter.
import frc.robot.commands.shooterComp.ShooterHold; // Command to hold the shooter in place.
import frc.robot.commands.shooterComp.ShooterStop; // Command to stop the shooter.

public class FeedAndHoldNote extends SequentialCommandGroup {

    // Constructor to initialize the FeedAndHoldNote command group.
    public FeedAndHoldNote() {
        addCommands(
            new ShooterHold(), // Hold the shooter in its position.
            new WaitCommand(0.48), // Wait for 0.48 seconds.
            new RollIntakeInSlow(144), // Slowly roll the intake in for 144 units (e.g., rotations).
            // Run the OutIntake and InShooter commands in parallel.
            new ParallelCommandGroup(new OutIntake(), new InShooter()),
            // Stop the intake and shooter simultaneously.
            new ParallelCommandGroup(new StopIntake(), new ShooterStop())
        );
    }

    // Method to cancel the ongoing command group.
    void Callcancel() {
        this.cancel(); // Cancel the FeedAndHoldNote command group.
    }
}
