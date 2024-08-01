package frc.robot.commands.compositions;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.drivetrain.vision.NoteDetector;
import frc.robot.subsystems.intakeComp.Intake;

public class NotePresent extends Command {
    private final NoteDetector noteDetector;
    private final Intake intake_subsystem;
    private final SwerveDrive swerve_subsystem;
    private final int index;
    private final boolean useNotPresent;

    public NotePresent(NoteDetector noteDetector, Intake intakeSubsystem, SwerveDrive swerveSubsystem, int index, boolean useNotPresent) {
      this.noteDetector = noteDetector;
      this.intake_subsystem = intakeSubsystem;
      this.swerve_subsystem = swerveSubsystem;
      this.index = index;
      this.useNotPresent = useNotPresent;
    }

    private boolean almost_equal(Pose2d a, Pose2d b) {
      return Math.abs(Math.atan(a.getY()/a.getX())-Math.atan(b.getY()/b.getX())) < 10 && Math.abs(a.getTranslation().getDistance(b.getTranslation())) < 1.5; // deg and meter
    }

    @Override
    public boolean isFinished() {
      Pose2d curr_pose = this.swerve_subsystem.getPose();
      Pose2d ideal = new Pose2d(Constants.FieldConstants.kNOTE_ARR[index].getX(), Constants.FieldConstants.kNOTE_ARR[index].getY(), new Rotation2d()).relativeTo(curr_pose);
      Pose2d measured = new Pose2d(this.noteDetector.getNoteFieldRelativePose(), new Rotation2d()).relativeTo(curr_pose);

      boolean present = (intake_subsystem.inIntake() || this.noteDetector.notePresent()) && (almost_equal(ideal, measured));
      Logger.recordOutput("NotePresent", present);

      if (useNotPresent) {
        return present;
      }
      return !present;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

