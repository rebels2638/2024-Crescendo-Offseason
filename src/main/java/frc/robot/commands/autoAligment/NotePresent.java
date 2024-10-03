package frc.robot.commands.autoAligment;

import org.littletonrobotics.junction.Logger; // For logging information.

import edu.wpi.first.math.geometry.Pose2d; // 2D pose representation.
import edu.wpi.first.math.geometry.Rotation2d; // Representation of rotation.
import edu.wpi.first.wpilibj2.command.Command; // Base class for commands.
import frc.robot.Constants; // Contains robot constants.
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive; // Swerve drive subsystem.
import frc.robot.subsystems.drivetrain.vision.NoteDetector; // Vision subsystem for detecting notes.
import frc.robot.subsystems.intakeComp.Intake; // Intake subsystem for handling objects.

public class NotePresent extends Command {

    private final NoteDetector noteDetector; // Instance for note detection.
    private final Intake intakeSubsystem; // Instance for controlling the intake.
    private final SwerveDrive swerveSubsystem; // Instance for swerve drive control.
    private final int index; // Index for identifying the note.
    private final boolean useNotPresent; // Flag indicating whether to use 'not present' logic.

    // Constructor to initialize dependencies.
    public NotePresent(NoteDetector noteDetector, Intake intakeSubsystem, SwerveDrive swerveSubsystem, int index, boolean useNotPresent) {
        this.noteDetector = noteDetector;
        this.intakeSubsystem = intakeSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.index = index;
        this.useNotPresent = useNotPresent;
    }

    // Helper method to check if two poses are almost equal.
    private static boolean almost_equal(Pose2d a, Pose2d b) {
        double rot = Math.abs(Math.atan(a.getY() / a.getX()) - Math.atan(b.getY() / b.getX())); // Calculate rotation difference.
        Logger.recordOutput("NotePresent/almostRot", rot); // Log rotation difference.

        // Check if the rotation and distance between poses are within thresholds.
        return rot < Math.toRadians(20) && Math.abs(a.getTranslation().getDistance(b.getTranslation())) <= 1.5; // Degrees and meters.
    }

    // Static method to determine if a note is present based on sensor input and robot position.
    public static boolean notePresent(NoteDetector noteDetector, Intake intakeSubsystem, SwerveDrive swerveSubsystem, int index, boolean useNotPresent) {
        Pose2d curr_pose = swerveSubsystem.getPose(); // Get current pose of the swerve drive.
        Pose2d ideal = new Pose2d(Constants.FieldConstants.kNOTE_ARR[index].toTranslation2d(), new Rotation2d()).relativeTo(curr_pose); // Get ideal pose based on note's position.
        Pose2d measured = new Pose2d(noteDetector.getNoteFieldRelativePose(), new Rotation2d()).relativeTo(curr_pose); // Get measured pose from note detector.

        // Calculate the rotation difference to the ideal pose.
        double rotDelta = Math.abs(Math.PI + Math.atan2((Constants.FieldConstants.kNOTE_ARR[index].getY() - curr_pose.getTranslation().getY()),
                            (Constants.FieldConstants.kNOTE_ARR[index].getX() - curr_pose.getTranslation().getX()))) - curr_pose.getRotation().getRadians();

        // Adjust rotation delta if it exceeds π radians.
        if (rotDelta >= Math.PI) {
            rotDelta = Math.PI * 2 - rotDelta;
        }

        Logger.recordOutput("NotePresent/rotDeltaDeg", Math.toDegrees(rotDelta)); // Log rotation delta in degrees.

        double dist = curr_pose.getTranslation().getDistance(Constants.FieldConstants.kNOTE_ARR[index].toTranslation2d()); // Calculate distance to ideal.
        Logger.recordOutput("NotePresent/distMeters", dist); // Log distance.

        // Determine if the note is present based on rotation, distance, and note detection state.
        boolean present = rotDelta >= Math.toRadians(20) || dist >= 2 || (noteDetector.hasTargets() && almost_equal(ideal, measured));

        // If the intake is active, set the present status to false.
        if (intakeSubsystem.inIntake()) {
            present = false;
        }

        Logger.recordOutput("NotePresent", present); // Log the presence state.

        // Mark the note as checked if it’s not present.
        if (!present) {
            noteDetector.setCheked(index);
        }

        // Return presence status based on the useNotPresent flag.
        return useNotPresent ? present : present;
    }

    @Override
    public boolean isFinished() { // Determine if the command is finished.
        return !notePresent(noteDetector, intakeSubsystem, swerveSubsystem, index, useNotPresent);
    }

    @Override
    public void end(boolean interrupted) { // Handle cleanup after command ends.
        Logger.recordOutput("NotePresent/interrupted", interrupted); // Log if the command was interrupted.
    }
}
