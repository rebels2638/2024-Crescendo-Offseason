package frc.robot.commands.compositions;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.drivetrain.vision.NoteDetector;
import frc.robot.subsystems.intakeComp.Intake;

public class Far1Present extends Command {
    private final NoteDetector noteDetector;
    private final Intake intake_subsystem;
    private final SwerveDrive swerve_subsystem;

    public Far1Present(NoteDetector noteDetector, Intake intakeSubsystem, SwerveDrive swerveSubsystem) {
      this.noteDetector = noteDetector;
      this.intake_subsystem = intakeSubsystem;
      this.swerve_subsystem = swerveSubsystem;
      addRequirements(noteDetector, intakeSubsystem, swerveSubsystem);
    }

    private boolean almost_equal(Pose2d a, Pose2d b) {
      return Math.abs(Math.atan(a.getY()/a.getX())-Math.atan(b.getY()/b.getX())) < 10 && a.getTranslation().getDistance(b.getTranslation()) < 1.5; // deg and meter
    }

    @Override
    public boolean isFinished() {
      Pose2d curr_pose = this.swerve_subsystem.getPose();
      Pose2d ideal = new Pose2d(Constants.FieldConstants.kNOTE_ARR[0].getX(), Constants.FieldConstants.kNOTE_ARR[0].getY(), new Rotation2d()).relativeTo(curr_pose);
      Pose2d measured = new Pose2d(this.noteDetector.getNoteFieldRelativePose(), new Rotation2d()).relativeTo(curr_pose);

      return (Intake.getInstance().inIntake() || this.noteDetector.notePresent()) && (almost_equal(ideal, measured));
    }

    @Override
    public void end(boolean interrupted) {

    }
}

