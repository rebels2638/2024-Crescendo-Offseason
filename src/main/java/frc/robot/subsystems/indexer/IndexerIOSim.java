package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.drivetrain.vision.NoteDetector;
import frc.robot.subsystems.intake.Intake;

public class IndexerIOSim implements IndexerIO {

    private final SwerveDrive swerveDrive;
    private final Intake intake;
    private final NoteDetector noteDetector;
    
    private boolean contact = false;
    private double initialIntakePose = 0;

    public IndexerIOSim(SwerveDrive swerveDrive, Intake intake, NoteDetector noteDetector) {
        this.swerveDrive = swerveDrive;
        this.intake = intake;
        this.noteDetector = noteDetector;
    }
    
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        Rotation2d robotYaw = swerveDrive.getPose().getRotation();
        Translation2d intakeTranslation2d = Constants.IntakeConstants.KINTAKE_TRANSLATION3D.toTranslation2d();
        intakeTranslation2d = intakeTranslation2d.rotateBy(robotYaw);
        intakeTranslation2d = intakeTranslation2d.plus(swerveDrive.getPose().getTranslation());

        Translation3d intakeTranslation3d = new Translation3d(
                                                    intakeTranslation2d.getX(), 
                                                    intakeTranslation2d.getY(), 
                                                    Constants.IntakeConstants.KINTAKE_TRANSLATION3D.getZ());
       
        
        Logger.recordOutput("Indexer/intakePose", new Pose3d(intakeTranslation3d, new Rotation3d()));

        // Translation3d closestNote = new Translation3d();
        // double minDist = Double.MAX_VALUE;
        // for (int i = 0; i < Constants.FieldConstants.kNOTE_ARR.length; i++) {
        //     double dist = intakeTranslation3d.getDistance(Constants.FieldConstants.kNOTE_ARR[i]);
        //     if (dist < minDist) {
        //         minDist = dist;
        //         closestNote = Constants.FieldConstants.kNOTE_ARR[i];
        //     }
        // }

        double dist = Double.MAX_VALUE;
        if (/*noteDetector.hasTargets()*/ true) {
            dist = noteDetector.getNoteFeildRelativePose().getDistance(intakeTranslation3d);
        }

        if (dist <= .1 && !contact && intake.getVelocityMps() >= .2) {
            contact = true;
            initialIntakePose = intake.getPoseMeters();
        }
        if (contact && intake.getPoseMeters() < initialIntakePose) {
            contact = false;
            inputs.inIntake = false;
        }
        if (contact && intake.getPoseMeters() - initialIntakePose >= Constants.FieldConstants.kNOTE_DIAMETER_METERS) {
            inputs.inIntake = true;
        }

        Logger.recordOutput("Indexer/contact", contact);
        Logger.recordOutput("Indexer/dist", dist);
        Logger.recordOutput("Indexer/initialIntakePose", initialIntakePose);


    }
}
