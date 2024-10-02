package frc.robot.commands.shooterComp;

import frc.robot.lib.input.XboxController; // Xbox controller for input.
import edu.wpi.first.wpilibj2.command.Command; // Base command class.
import frc.robot.subsystems.shooterComp.Shooter; // Shooter subsystem reference.

public class ShooterToggle extends Command {

    private final double stopVelocitySetPoint = 0; // Velocity to stop the shooter.
    private final double windupVelocitySetpoint = 5; // Velocity for winding up.
    private final double windupReverseVelocitySetpoint = -2; // Velocity for reverse windup.
    private final double holdVelocitySetpoint = 0; // Hold velocity.

    private int tapped; // Counter for button taps.

    private final Shooter shooterSubsystem = Shooter.getInstance(); // Singleton instance of the shooter subsystem.
    private XboxController m_controller; // Controller instance.

    public ShooterToggle(XboxController controller) {
        this.m_controller = controller; // Initialize the controller.
        this.tapped = 0; // Initialize tap counter.
    }

    @Override
    public void execute() {
        if (this.m_controller.getLeftBumper().getAsBoolean()) { // Check if the left bumper is pressed.
            this.tapped++; // Increment tap counter.
            double desiredSpeed = 0; // Variable to hold the desired speed.

            // Determine the desired speed based on the number of taps.
            switch (tapped) {
                case 1:
                    desiredSpeed = this.windupVelocitySetpoint; // Set for windup.
                    break;

                case 2:
                    desiredSpeed = this.stopVelocitySetPoint; // Set to stop the shooter.
                    break;

                case 3:
                    desiredSpeed = this.windupReverseVelocitySetpoint; // Set for reverse windup.
                    break;

                case 4:
                    desiredSpeed = this.holdVelocitySetpoint; // Set for holding position.
                    this.tapped = 0; // Reset counter after the fourth tap.
                    break;
            }

            // Apply the desired speed to the shooter.
            shooterSubsystem.setVelocityRadSec(desiredSpeed, false, 0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return true; // Command finishes immediately after execution.
    }
}
