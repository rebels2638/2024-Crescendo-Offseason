package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.RollIntakeIn;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.commands.shooterComp.ShooterStop;
import frc.robot.commands.shooterComp.ShooterWindup;

public class ShootNoteAuto extends SequentialCommandGroup {
    public ShootNoteAuto() {
        addCommands(
            // new WaitCommand(0.2),
           // new PivotTurtle(),
            // new WaitCommand(0.25), unneeded
            new RollIntakeIn(),
            new WaitCommand(0.22),
            new StopIntake()
        );
    }
}
