package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.StopDrive;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

public class CancelIntakeNote extends SequentialCommandGroup {
    public CancelIntakeNote(SequentialCommandGroup c, SequentialCommandGroup c2, SwerveDrive swerveDrive) {
        if(c != null && c2 != null){
        addCommands(
            new StopDrive(swerveDrive),
            new InstantCommand(()->c.cancel()),
            new InstantCommand(()->c2.cancel()),
                new StopIntake(),
                new PivotTurtle());
        }
        else if(c != null){
            addCommands(
                new StopDrive(swerveDrive),
                new InstantCommand(()->c.cancel()),
                new StopIntake(), 
                new PivotTurtle()
                );
        }
        else if(c2 != null)
        {
            addCommands(
                new StopDrive(swerveDrive),
                new InstantCommand(()->c2.cancel()),
                new StopIntake(), 
                new PivotTurtle()
                );
        }
        else{
            addCommands(
                new StopDrive(swerveDrive),
                new StopIntake(), 
                new PivotTurtle()
                );
        }
    }
}