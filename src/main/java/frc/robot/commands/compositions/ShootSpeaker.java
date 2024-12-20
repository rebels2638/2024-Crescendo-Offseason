package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.commands.shooterComp.ShooterStop;
// import frc.robot.commands.shooter.AlignWithTargetPoint;
import frc.robot.commands.intake.RollIntakeIn;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.intake.InIntake;
import frc.robot.commands.elevator.MoveElevatorTurtle;
import frc.robot.commands.pivot.PivotTurtle;

// import frc.robot.subsystems.limelight.PoseLimelight;

public class ShootSpeaker extends SequentialCommandGroup {
    public ShootSpeaker(/*PoseLimelight visionSubsystem SwerveSubsystem swerveSubsystem*/) {
        addCommands(
            // new ParallelRaceGroup(new AlignWithTargetPoint(swerveSubsystem, visionSubsystem), new ShooterWindup(RebelUtil.flywheelSpeed)),
            new ParallelCommandGroup(new MoveElevatorTurtle(), new PivotTurtle()),
            new ParallelRaceGroup(new RollIntakeIn(), new InIntake()),
            new StopIntake(),
            new ShooterStop()
        );
    }
}
