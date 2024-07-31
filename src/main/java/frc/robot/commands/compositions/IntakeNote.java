package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.DriveToNote;
import frc.robot.commands.intake.InIntake;
import frc.robot.commands.intake.OutIntake;
import frc.robot.commands.intake.RollIntakeIn;
import frc.robot.commands.intake.RollIntakeInSlow;
import frc.robot.commands.intake.RollIntakeOut;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.pivot.PivotToTorus;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.drivetrain.vision.NoteDetector;
import frc.robot.subsystems.intakeComp.Intake;

public class IntakeNote extends SequentialCommandGroup {
    public IntakeNote(SwerveDrive swerveDrive, Intake intakeSubsystem, NoteDetector noteDetector) {
        addCommands(
            new ParallelCommandGroup(
                new RollIntakeIn(),
                new PivotToTorus()
            ),
            new DriveToNote(swerveDrive, intakeSubsystem, noteDetector),
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
        );

    }


}
