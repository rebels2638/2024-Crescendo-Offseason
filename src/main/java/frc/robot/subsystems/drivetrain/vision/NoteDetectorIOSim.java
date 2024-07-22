package frc.robot.subsystems.drivetrain.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

public class NoteDetectorIOSim implements NoteDetectorIO {
    private final SwerveDrive swerveDrive;
    public NoteDetectorIOSim(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }
    @Override
    public void updateInputs(NoteDetectorIOInputs inputs) {
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
        
        Logger.recordOutput("NoteDetector/cameraPose", cameraPose);

        double vxAngle = (Math.atan2(Constants.FieldConstants.kNOTE_3_TRANS.getY() - cameraPose.getY(), 
                                    Constants.FieldConstants.kNOTE_3_TRANS.getX() - cameraPose.getX()) - 
                                    cameraPose.getRotation().getZ()) % (Math.PI * 2);

        double x = Math.sqrt(Math.pow(Constants.FieldConstants.kNOTE_3_TRANS.getY() - cameraPose.getY(), 2) +
                                Math.pow(Constants.FieldConstants.kNOTE_3_TRANS.getX() - cameraPose.getX(), 2));

        double vyAngle = (Math.atan2(Constants.FieldConstants.kNOTE_3_TRANS.getZ() - cameraPose.getZ(), x) + 
                        cameraPose.getRotation().getY()) % (Math.PI * 2);
        
        double dist = cameraPose.getTranslation().getDistance(Constants.FieldConstants.kNOTE_3_TRANS);

        Logger.recordOutput("NoteDetector/vxAngle", vxAngle);
        Logger.recordOutput("NoteDetector/vyAngle", vyAngle);

        inputs.hasTargets = false;
        if (Math.abs(vxAngle) <= Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_FOV_X_RAD/2 &&
            Math.abs(vyAngle) <= Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_FOV_Y_RAD/2 &&
            dist <= Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_MAX_RANGE_METERS) {
            inputs.hasTargets = true;
            inputs.vxRadians = vxAngle;
            inputs.vyRadians = vyAngle;
        }

    }
}
