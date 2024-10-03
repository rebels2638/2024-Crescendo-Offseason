package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.RollIntakeIn;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooterComp.ShooterStop;
import frc.robot.commands.shooterComp.ShooterWindup;

/**
 * A command group that represents shooting a note.
 */
public class ShootNote extends SequentialCommandGroup {
    public ShootNote() {
        addCommands(
            new ShooterWindup(),
            new WaitCommand(0.70), //Modify this later on.
            new RollIntakeIn(),
            new WaitCommand(0.4),
            new StopIntake(),
            new ShooterStop()
        );
    }


}
