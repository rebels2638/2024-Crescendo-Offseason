package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.RollIntakeIn;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooterComp.ShooterStop;

public class ShootNoteTele extends SequentialCommandGroup {
    public ShootNoteTele() {
        addCommands(
             //Modify this later on.
            new RollIntakeIn(),
            new WaitCommand(0.55),
            new StopIntake(),
            new ShooterStop()
        );
    }


}
