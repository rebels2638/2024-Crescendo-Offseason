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
      return Math.abs(Math.atan(a.getY()/a.getX())-Math.atan(b.getY()/b.getX())) < Math.toRadians(20) && Math.abs(a.getTranslation().getDistance(b.getTranslation())) <= 1.5; // deg and meter
    }

    @Override
    public boolean isFinished() {
      if (intake_subsystem.inIntake()) {
        return true;
      }
      
      Pose2d curr_pose = this.swerve_subsystem.getPose();
      Pose2d ideal = new Pose2d(Constants.FieldConstants.kNOTE_ARR[index].toTranslation2d(), new Rotation2d()).relativeTo(curr_pose);
      Pose2d measured = new Pose2d(this.noteDetector.getNoteFieldRelativePose(), new Rotation2d()).relativeTo(curr_pose);

      double rotDelta = Math.abs(-Math.atan2((Constants.FieldConstants.kNOTE_ARR[index].getY() - curr_pose.getTranslation().getY()),  
      (Constants.FieldConstants.kNOTE_ARR[index].getX() - curr_pose.getTranslation().getX()))) + Math.PI - curr_pose.getRotation().getRadians();

      Logger.recordOutput("NotePresent/rotDelta", rotDelta);

      boolean present = 
        rotDelta >= Math.toRadians(20) ||
        curr_pose.getTranslation().getDistance(Constants.FieldConstants.kNOTE_ARR[index].toTranslation2d()) >= 1.2 ||
        (this.noteDetector.hasTargets() && almost_equal(ideal, measured));

     
      Logger.recordOutput("NotePresent", present);

      if (useNotPresent) {
        return present;
      }
      return !present;
    }

    @Override
    public void end(boolean interrupted) {
      Logger.recordOutput("NotePresent/interrupted", interrupted);
    }
}

