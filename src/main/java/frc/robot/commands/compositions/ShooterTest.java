package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooterComp.ShooterHold;
import frc.robot.commands.shooterComp.ShooterStop;

/**
 * This class represents a test sequence for the shooter mechanism.
 * It consists of a series of commands executed sequentially.
 */
public class ShooterTest extends SequentialCommandGroup {
    public ShooterTest() {
        addCommands(
            new ParallelRaceGroup(
                new ShooterHold(),
                new WaitCommand(0.2)),
                new ShooterStop()
        );
    }
}