package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.OutIntake;
import frc.robot.commands.intake.RollIntakeInSlow;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooterComp.InShooter;
import frc.robot.commands.shooterComp.ShooterHold;
import frc.robot.commands.shooterComp.ShooterStop;

public class FeedAndHoldNote extends SequentialCommandGroup {
    public FeedAndHoldNote() {
        addCommands(                                                                                                                                                                                           

            new ShooterHold(),
            new WaitCommand(0.48),
            new RollIntakeInSlow(144),
            new ParallelCommandGroup(new OutIntake(), new InShooter()),
            new ParallelCommandGroup(new StopIntake(), new ShooterStop()));
    }
    void Callcancel(){
        this.cancel();
    }


}