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
import frc.robot.subsystems.drivetrain.vision.NoteDetectorIOInputsAutoLogged;

public class NoteDetector extends SubsystemBase {
    private final NoteDetectorIOInputsAutoLogged inputs = new NoteDetectorIOInputsAutoLogged();
    private NoteDetectorIO io;
    private final SwerveDrive swerveDrive;
    public NoteDetector(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        switch(Constants.currentMode) {
            case SIM:
                io = new NoteDetectorIOSim(swerveDrive);
                break;
            default: 
                io = new NoteDetectorIO() {};
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
        if (!inputs.hasTargets) {return new Translation3d();}

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
        Logger.recordOutput("NoteDetector/cameraRot", cameraPose.getRotation().getY());


        Translation3d sample = new Translation3d(dist * Math.cos(inputs.vxRadians), dist * Math.sin(inputs.vxRadians), dist * Math.sin(inputs.vyRadians));
        sample = sample.rotateBy(cameraPose.getRotation());
        sample = sample.plus(cameraPose.getTranslation());

        return sample;
    }
}
