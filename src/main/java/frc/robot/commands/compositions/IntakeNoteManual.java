package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.InIntake;
import frc.robot.commands.intake.OutIntake;
import frc.robot.commands.intake.RollIntakeIn;
import frc.robot.commands.intake.RollIntakeInSlow;
import frc.robot.commands.intake.RollIntakeOut;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.pivot.PivotToTorus;
import frc.robot.commands.pivot.PivotTurtle;

public class IntakeNoteManual extends SequentialCommandGroup {
    public IntakeNoteManual() {
        addCommands(
            // new InstantCommand(()-> Intake.getInstance().setIntakeStatus(true)),
            new PivotToTorus(),  
            new RollIntakeIn(),
            new InIntake(),
            new StopIntake(),
            new PivotTurtle(),
            new RollIntakeOut(), 
            new WaitCommand(0.15),
            new OutIntake(),
            new StopIntake(), 
            new RollIntakeInSlow(),
            new InIntake(),
            new WaitCommand(0.1),
            new StopIntake()
            // new InstantCommand(()-> Intake.getInstance().setIntakeStatus(false))
            // new LEDController(0.77) // green
        );

    }

    void Callcancel(){
        this.cancel();
    }


}