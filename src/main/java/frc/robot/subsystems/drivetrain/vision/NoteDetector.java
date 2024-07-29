package frc.robot.subsystems.drivetrain.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

public class NoteDetector extends SubsystemBase {
    private final NoteDetectorIOInputsAutoLogged inputs = new NoteDetectorIOInputsAutoLogged();
    private NoteDetectorIO io;
    private final SwerveDrive swerveDrive;

    private Translation2d prevSample = new Translation2d();

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

        Logger.recordOutput("NoteDetector/estimNotePose", new Translation3d(getNoteFeildRelativePose().getX(), getNoteFeildRelativePose().getY(),0));
    }

    public boolean hasTargets() {
        return inputs.hasTargets;
    }

    public Translation2d getNoteFeildRelativePose() {
        if (!inputs.hasTargets) { return prevSample; }

        double pitch = Math.PI / 2 - (Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_POSE.getRotation().getY() - inputs.tyRadians);
        double yaw = inputs.txRadians;

        Logger.recordOutput("NoteDetector/pitchDeg", Math.toDegrees(pitch));
        Logger.recordOutput("NoteDetector/yawDeg", Math.toDegrees(yaw));

        double xMeters = Math.tan(pitch) * Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_POSE.getZ();
        double yMeters = xMeters * Math.tan(yaw);

        Logger.recordOutput("NoteDetector/xMeters", xMeters);
        Logger.recordOutput("NoteDetector/yMeters", yMeters);


        Translation2d realtiveTranslation2d = new Translation2d(xMeters, yMeters);
        Translation2d absoluteTranslation2d = realtiveTranslation2d.
            minus(Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_POSE.getTranslation().toTranslation2d()).
            rotateBy(
                new Rotation2d(Constants.VisionConstants.kNOTE_DETECTOR_CAMERA_POSE.getRotation().getZ())).
            rotateBy(
                new Rotation2d(swerveDrive.getPose().getRotation().getRadians()))/* .
            minus(swerveDrive.getPose().getTranslation())*/;
        
        absoluteTranslation2d = swerveDrive.getPose().getTranslation().minus(absoluteTranslation2d);
        prevSample = absoluteTranslation2d;
        return absoluteTranslation2d;
    }

    public double getDrivetrainDistToNote() {
        return swerveDrive.getPose().getTranslation().getDistance(getNoteFeildRelativePose());
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
       
        return intakeTranslation3d.getDistance(new Translation3d(getNoteFeildRelativePose().getX(), getNoteFeildRelativePose().getY(), 0));
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
