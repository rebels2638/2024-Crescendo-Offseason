package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.InIntake;
import frc.robot.commands.intake.OutIntake;
import frc.robot.commands.intake.RollIntakeIn;
import frc.robot.commands.intake.RollIntakeOut;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.pivot.PivotMidway;
import frc.robot.commands.pivot.PivotToTorus;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.commands.intake.RollIntakeInSlow;

public class IntakeNoteAuto extends SequentialCommandGroup {
    public IntakeNoteAuto() {
        addCommands(
            new StopIntake(),
            new PivotToTorus(),
            new RollIntakeIn(),
            new InIntake(),
            new StopIntake(),
            new PivotMidway(),
            new RollIntakeOut(), 
            new OutIntake(),
            new StopIntake(), 
            new RollIntakeInSlow(), 
            new WaitCommand(0.1),
            new StopIntake(),
            new PivotTurtle()
        );
    }
}