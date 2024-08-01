package frc.robot.subsystems.drivetrain.vision;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

public class NoteDetector extends SubsystemBase {
    private final NoteDetectorIOInputsAutoLogged inputs = new NoteDetectorIOInputsAutoLogged();
    private NoteDetectorIO io;
    private final SwerveDrive swerveDrive;

    private Translation2d prevSample = new Translation2d();

    private boolean[] checked = new boolean[] {
        false,
        false,
        false,
        false,
        false,
        true,
        false,
        false
    };

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

        Logger.recordOutput("NoteDetector/estimNotePose", new Translation3d(getNoteFieldRelativePose().getX(), getNoteFieldRelativePose().getY(),0));
    }

    public boolean hasTargets() {
        return inputs.hasTargets;
    }

    public Translation2d getNoteFieldRelativePose() {
        if (!inputs.hasTargets) { return prevSample; }

        double pitch = Math.PI / 2 - (Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_POSE.getRotation().getY() - inputs.tyRadians);
        double yaw = inputs.txRadians;

        Logger.recordOutput("NoteDetector/pitchDeg", Math.toDegrees(pitch));
        Logger.recordOutput("NoteDetector/yawDeg", Math.toDegrees(yaw));

        double xMeters = Math.tan(pitch) * Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_POSE.getZ();
        double yMeters = xMeters * Math.tan(yaw);

        Logger.recordOutput("NoteDetector/xMeters", xMeters);
        Logger.recordOutput("NoteDetector/yMeters", yMeters);

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


        Translation2d realtiveTranslation2d = new Translation2d(xMeters, yMeters);
        Translation2d absoluteTranslation2d = realtiveTranslation2d.
            rotateBy(new Rotation2d(cameraPose.getRotation().getZ()));
        
        absoluteTranslation2d = swerveDrive.getPose().getTranslation().minus(absoluteTranslation2d);
        prevSample = absoluteTranslation2d;

        // TODO: Fixed sim :skull:
        if (Constants.currentMode == Constants.Mode.SIM) {
            Translation2d closestNote = new Translation2d();
            double minDist = Double.MAX_VALUE;
            for (int i = 0; i < Constants.FieldConstants.kNOTE_ARR.length; i++) {

                var alliance = DriverStation.getAlliance();
                Pose2d notePose = new Pose2d(Constants.FieldConstants.kNOTE_ARR[i].toTranslation2d(), new Rotation2d());
                // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                //     notePose = GeometryUtil.flipFieldPose(notePose);
                // } 

                double dist = cameraPose.getTranslation().toTranslation2d().getDistance(notePose.getTranslation());
                if (dist < minDist) {
                    minDist = dist;
                    closestNote = notePose.getTranslation();
                }
            }

            return closestNote;
        }

        return absoluteTranslation2d;
    }

    public double getDrivetrainDistToNote() {
        return swerveDrive.getPose().getTranslation().getDistance(getNoteFieldRelativePose());
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
       
        return intakeTranslation3d.getDistance(new Translation3d(getNoteFieldRelativePose().getX(), getNoteFieldRelativePose().getY(), 0));
    }

    private boolean almost_equal(Pose2d a, Pose2d b) {
        return Math.abs(Math.atan(a.getY()/a.getX())-Math.atan(b.getY()/b.getX())) < Math.toRadians(20) && Math.abs(a.getTranslation().getDistance(b.getTranslation())) <= 1.5; // deg and meter
    }

    public boolean notePresent(int index) {
        Logger.recordOutput("NoteDetector/intakeDistToNote", intakeDistToNote());
        Logger.recordOutput("NoteDetector/drivetrainDistToNote", getDrivetrainDistToNote());

        Pose2d curr_pose = swerveDrive.getPose();
        Pose2d ideal = new Pose2d(Constants.FieldConstants.kNOTE_ARR[index].toTranslation2d(), new Rotation2d()).relativeTo(curr_pose);
        Pose2d measured = new Pose2d(getNoteFieldRelativePose(), new Rotation2d()).relativeTo(curr_pose);
  
        double rotDelta = Math.abs(Math.atan2((Constants.FieldConstants.kNOTE_ARR[index].getY() - curr_pose.getTranslation().getY()),  
        (Constants.FieldConstants.kNOTE_ARR[index].getX() - curr_pose.getTranslation().getX()))) + curr_pose.getRotation().getRadians();
  
        Logger.recordOutput("NoteDetector/notePresent/rotDelta", rotDelta);
  
        boolean present = 
          (rotDelta >= Math.toRadians(20) &&
          curr_pose.getTranslation().getDistance(Constants.FieldConstants.kNOTE_ARR[index].toTranslation2d()) >= 2) ||
          (hasTargets() && almost_equal(ideal, measured));
  
       
        Logger.recordOutput("NoteDetector/notePresent", present);
        
        checked[index] = true;
        return present;
    }

    public boolean checked(int index) {
        return checked[index];
    }
}
