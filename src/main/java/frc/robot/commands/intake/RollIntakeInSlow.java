package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command; // Base class for commands in WPILib.
import frc.robot.subsystems.intakeComp.Intake; // This is the intake system.

public class RollIntakeInSlow extends Command {

    private final Intake intakeSubsystem = Intake.getInstance(); // Get the intake system.
    private double velo; // Variable to hold the intake speed.

    // Default constructor sets the speed to 150.
    public RollIntakeInSlow() {
        velo = 150; // Default speed for slow intake.
    }

    // This constructor allows setting a custom speed.
    public RollIntakeInSlow(double a) {
        velo = a; // Set the intake speed to the value provided.
    }

    // This runs when the command starts.
    @Override
    public void initialize() {
        // Set the intake to spin at the specified slow speed.
        intakeSubsystem.setVelocityRadSec(Math.toRadians(velo)); // Convert speed to radians.
    }

    // This runs when the command ends or is stopped.
    @Override
    public void end(boolean isInterrupted) {
        // Optionally stop the intake when the command ends (currently not active).
        // intakeSubsystem.setVelocityRadSec(0); // Stop spinning.
    }

    // This checks if the command is done.
    @Override
    public boolean isFinished() {
        return true; // The command is done immediately.
    }
}
