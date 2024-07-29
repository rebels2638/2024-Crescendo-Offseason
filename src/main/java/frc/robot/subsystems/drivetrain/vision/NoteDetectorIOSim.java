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

        Translation3d closestNote = new Translation3d();
        double minDist = Double.MAX_VALUE;
        for (int i = 0; i < Constants.FieldConstants.kNOTE_ARR.length; i++) {
            double dist = cameraPose.getTranslation().getDistance(Constants.FieldConstants.kNOTE_ARR[i]);
            if (dist < minDist) {
                minDist = dist;
                closestNote = Constants.FieldConstants.kNOTE_ARR[i];
            }
        }
        double vxAngle = ((Math.atan2(closestNote.getY() - cameraPose.getY(), 
                                    closestNote.getX() - cameraPose.getX()) - 
                                    cameraPose.getRotation().getZ())) % (2 * Math.PI);

        double x = Math.sqrt(Math.pow(closestNote.getY() - cameraPose.getY(), 2) +
                                Math.pow(closestNote.getX() - cameraPose.getX(), 2));

        double vyAngle = ((Math.atan2(closestNote.getZ() - cameraPose.getZ(), x) + 
                            cameraPose.getRotation().getY())) % (2 * Math.PI);
        

        if (vxAngle <= -Math.PI) {
            vxAngle += 2 * Math.PI;
        }
        else if (vxAngle >= 180) {
            vxAngle -= 2 * Math.PI;
        }
        Logger.recordOutput("NoteDetector/txAngleDeg", Math.toDegrees(vxAngle));
        Logger.recordOutput("NoteDetector/tyAngleDeg", Math.toDegrees(vyAngle));

        inputs.hasTargets = false;
        if (Math.abs(vxAngle) <= Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_FOV_X_RAD/2 &&
            Math.abs(vyAngle) <= Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_FOV_Y_RAD/2 &&
            minDist <= Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_MAX_RANGE_METERS) {
            inputs.hasTargets = true;
            inputs.txRadians = -vxAngle;
            inputs.tyRadians = vyAngle;
        }

    }
}
