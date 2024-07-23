package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeNote extends Command {
    private Intake intakeSubsystem;
    public IntakeNote(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setSpeedMps(.5);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.inIntake();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeedMps(0);
    }
}
