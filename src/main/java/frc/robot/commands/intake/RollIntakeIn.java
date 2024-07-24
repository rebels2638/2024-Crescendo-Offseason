package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeComp.Intake;
import frc.robot.subsystems.pivotComp.Pivot;

public class RollIntakeIn extends Command {
    private final Intake intakeSubsystem = Intake.getInstance();
    private final Pivot pivotSubsystem = Pivot.getInstance();
    
    public RollIntakeIn() {
    }

    @Override
    public void initialize() { 
        if (pivotSubsystem.getDegAngle() < 45) {
            intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 10)); //Use radians directly.
        } else {
            intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 8)); //Use radians directly
        }
    }
    
    @Override
    public void end(boolean isInterrupted){
        // intakeSubsystem.setVelocityRadSec(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
