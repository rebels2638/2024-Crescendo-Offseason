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
    private final boolean[] taken = new boolean[] {true, true, false, false, false, false, false, false};
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

        double minDist = Double.MAX_VALUE;
        inputs.hasTargets = false;
        for (int i = 0; i < Constants.FieldConstants.kNOTE_ARR.length; i++) {
            if (taken[i]) {
                inputs.hasTargets = false;
                continue;
            }
    
            Translation3d closestNote = Constants.FieldConstants.kNOTE_ARR[i];
            double vxAngle = ((cameraPose.getRotation().getZ() - 
                        Math.atan2(closestNote.getY() - cameraPose.getY(), 
                        closestNote.getX() - cameraPose.getX()))) % (2 * Math.PI);

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

            Logger.recordOutput("NoteDetector/dist", closestNote.getDistance(cameraPose.getTranslation()));

            if (Math.abs(vxAngle) <= Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_FOV_X_RAD/2 &&
                Math.abs(vyAngle) <= Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_FOV_Y_RAD/2 &&
                closestNote.getDistance(cameraPose.getTranslation()) <= Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_MAX_RANGE_METERS) {
                inputs.hasTargets = true;
                inputs.txRadians = -vxAngle;
                inputs.tyRadians = vyAngle;
                if (closestNote.getDistance(cameraPose.getTranslation()) < minDist) {
                    inputs.hasTargets = true;
                    inputs.bestNote = closestNote.toTranslation2d();
                    minDist = closestNote.getDistance(cameraPose.getTranslation());
                }
            }
            
        }
        

    }
}
