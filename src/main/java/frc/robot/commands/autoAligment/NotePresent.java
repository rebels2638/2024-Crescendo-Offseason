package frc.robot.commands.autoAligment;

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
    private final Intake intakeSubsystem;
    private final SwerveDrive swerveSubsystem;
    private final int index;
    private final boolean useNotPresent;

    public NotePresent(NoteDetector noteDetector, Intake intakeSubsystem, SwerveDrive swerveSubsystem, int index, boolean useNotPresent) {
      this.noteDetector = noteDetector;
      this.intakeSubsystem = intakeSubsystem;
      this.swerveSubsystem = swerveSubsystem;
      this.index = index;
      this.useNotPresent = useNotPresent;
    }

    private static boolean almost_equal(Pose2d a, Pose2d b) {
      double rot = Math.abs(Math.atan(a.getY()/a.getX())-Math.atan(b.getY()/b.getX()));
      Logger.recordOutput("NotePresent/almostRot", rot);
      return rot < Math.toRadians(20) && Math.abs(a.getTranslation().getDistance(b.getTranslation())) <= 1.5; // deg and meter
    }

    public static boolean notePresent(NoteDetector noteDetector, Intake intakeSubsystem, SwerveDrive swerveSubsystem, int index, boolean useNotPresent) {
      Pose2d curr_pose = swerveSubsystem.getPose();
      Pose2d ideal = new Pose2d(Constants.FieldConstants.kNOTE_ARR[index].toTranslation2d(), new Rotation2d()).relativeTo(curr_pose);
      Pose2d measured = new Pose2d(noteDetector.getNoteFieldRelativePose(), new Rotation2d()).relativeTo(curr_pose);

      double rotDelta = Math.abs(Math.PI + Math.atan2((Constants.FieldConstants.kNOTE_ARR[index].getY() - curr_pose.getTranslation().getY()),  
      (Constants.FieldConstants.kNOTE_ARR[index].getX() - curr_pose.getTranslation().getX()))) - curr_pose.getRotation().getRadians();
      if (rotDelta >= Math.PI) {
        rotDelta = Math.PI * 2 - rotDelta;
      }

      Logger.recordOutput("NotePresent/rotDeltaDeg", Math.toDegrees(rotDelta));

      double dist = curr_pose.getTranslation().getDistance(Constants.FieldConstants.kNOTE_ARR[index].toTranslation2d());
      Logger.recordOutput("NotePresent/distMeters", dist);
      
      if (rotDelta > 180) {}
      boolean present = 
        rotDelta >= Math.toRadians(20) ||
        dist >= 2 ||
        (noteDetector.hasTargets() && almost_equal(ideal, measured));

     
      if (intakeSubsystem.inIntake()) {
        present = false;
      }
      
      Logger.recordOutput("NotePresent", present);

      if (!present) {
        noteDetector.setCheked(index);
      }

      if (useNotPresent) {
        return present;
      }
      return present;
    }

    @Override
    public boolean isFinished() {  
      return !notePresent(noteDetector, intakeSubsystem, swerveSubsystem, index, useNotPresent);
    }

    @Override
    public void end(boolean interrupted) {
      Logger.recordOutput("NotePresent/interrupted", interrupted);
    }
}

