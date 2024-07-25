package frc.robot.subsystems.drivetrain.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

public class NoteDetector extends SubsystemBase {
    private final NoteDetectorIOInputsAutoLogged inputs = new NoteDetectorIOInputsAutoLogged();
    private NoteDetectorIO io;
    private final SwerveDrive swerveDrive;

    private Translation3d prevSample = new Translation3d();

    public NoteDetector(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        switch(Constants.currentMode) {
            case SIM:
                io = new NoteDetectorIOSim(swerveDrive);
                break;
            default: 
                io = new NoteDetectorIOReal();
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("NoteDetector", inputs);

        Logger.recordOutput("NoteDetector/estimNotePose", getNoteFeildRelativePose());
    }

    public boolean hasTargets() {
        return inputs.hasTargets;
    }

    public Translation3d getNoteFeildRelativePose() {
        if (!inputs.hasTargets) { return prevSample; }

        Rotation2d robotYaw = swerveDrive.getPose().getRotation();
        Translation2d cameraTranslation2d = Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_POSE.getTranslation().toTranslation2d();
        cameraTranslation2d = cameraTranslation2d.rotateBy(robotYaw);
        cameraTranslation2d = cameraTranslation2d.plus(swerveDrive.getPose().getTranslation());

        Translation3d cameraTranslation = new Translation3d(
                                                    cameraTranslation2d.getX(), 
                                                    cameraTranslation2d.getY(), 
                                                    Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_POSE.getTranslation().getZ());
        Rotation3d cameraRotation = new Rotation3d(
                                                    Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_POSE.getRotation().getX(),
                                                    Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_POSE.getRotation().getY(),
                                                    Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_POSE.getRotation().getZ() + robotYaw.getRadians());
        
        Pose3d cameraPose = new Pose3d(cameraTranslation, cameraRotation);
        double dist = cameraPose.getZ() / Math.sin(cameraPose.getRotation().getY() - inputs.vyRadians);
        Logger.recordOutput("NoteDetector/dist", dist);
        Logger.recordOutput("NoteDetector/cameraPose", cameraPose);


        Translation3d sample = new Translation3d(
            dist * Math.cos(inputs.vxRadians), 
            dist * Math.sin(inputs.vxRadians), 
            dist * Math.sin(inputs.vyRadians));

        sample = sample.plus(cameraPose.getTranslation());
        sample = sample.rotateBy(cameraPose.getRotation());

        prevSample = sample;
        return sample;
    }

    public double getDrivetrainDistToNote() {
        return swerveDrive.getPose().getTranslation().getDistance(getNoteFeildRelativePose().toTranslation2d());
    }

    public double intakeDistToNote() {
        Rotation2d robotYaw = swerveDrive.getPose().getRotation();
        Translation2d intakeTranslation2d = Constants.IntakeConstants.KINTAKE_TRANSLATION3D.toTranslation2d();
        intakeTranslation2d = intakeTranslation2d.rotateBy(robotYaw);
        intakeTranslation2d = intakeTranslation2d.plus(swerveDrive.getPose().getTranslation());

        Translation3d intakeTranslation3d = new Translation3d(
                                                    intakeTranslation2d.getX(), 
                                                    intakeTranslation2d.getY(), 
                                                    Constants.IntakeConstants.KINTAKE_TRANSLATION3D.getZ());
       
        return intakeTranslation3d.getDistance(getNoteFeildRelativePose());
    }

    public boolean notePresent() {
        Logger.recordOutput("NoteDetector/intakeDistToNote", intakeDistToNote());
        Logger.recordOutput("NoteDetector/drivetrainDistToNote", getDrivetrainDistToNote());

        return (
            hasTargets() || 
            (intakeDistToNote() <= Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_BLIND_SPOT_DISTANCE_METERS && 
            intakeDistToNote() < getDrivetrainDistToNote())
        );
    }
}
