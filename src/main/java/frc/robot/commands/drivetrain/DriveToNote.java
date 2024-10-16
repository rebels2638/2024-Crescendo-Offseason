package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.util.RebelUtil;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.drivetrain.vision.NoteDetector;
import frc.robot.subsystems.intakeComp.Intake;

public class DriveToNote extends Command {
    private final Intake intake;
    private final NoteDetector noteDetector;
    private final SwerveDrive swerveDrive;

    private final PIDController m_translationalController;
    private final ProfiledPIDController m_rotationalController;

    private Rotation2d initialYaw = new Rotation2d();
    private boolean over = false;
    private double drivingSpeedMps = 1;

    public DriveToNote(SwerveDrive swerveDrive, Intake intakeSubsystem, NoteDetector noteDetector) {
        this.intake = intakeSubsystem;
        this.noteDetector = noteDetector;
        this.swerveDrive = swerveDrive;

        switch (Constants.currentMode) {
            case SIM:
                m_translationalController = new PIDController(1.2, 0, 0);
                m_rotationalController = new ProfiledPIDController(2, 0, 0,
                 new Constraints(Constants.Auton.MAX_ANGULAR_VELO_RPS * 2 * Math.PI, 
                 Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED * 2 * Math.PI));
                break;
        
            default:
                m_translationalController = new PIDController(1.6, 0, 0);
                m_rotationalController = new ProfiledPIDController(1.2, 0, 0,
                 new Constraints(Constants.Auton.MAX_ANGULAR_VELO_RPS * 2 * Math.PI, 
                 Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED * 2 * Math.PI));
                
                m_rotationalController.setTolerance(Math.toRadians(5));
                break;
        }
        m_rotationalController.enableContinuousInput(0, Math.PI * 2);
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        initialYaw = swerveDrive.getPose().getRotation();
        Logger.recordOutput("IntakeNOteCommand/initialYaw", initialYaw);
        Logger.recordOutput("IntakeNoteCommand/end", false);

        double currentSpeed = Math.sqrt(Math.pow(swerveDrive.getMeasuredFeildRelativeSpeeds().vxMetersPerSecond,2) + 
                                        Math.pow(swerveDrive.getMeasuredFeildRelativeSpeeds().vyMetersPerSecond,2));

        drivingSpeedMps = RebelUtil.constrain(currentSpeed, 1, 1.6);
        Logger.recordOutput("IntakeNoteCommand/drivingSpeedMps", drivingSpeedMps);
    }

    @Override
    public void execute() {
        Rotation2d robotYaw = swerveDrive.getPose().getRotation();
        Translation2d intakeTranslation2d = Constants.IntakeConstants.KINTAKE_TRANSLATION3D.toTranslation2d();
        intakeTranslation2d = intakeTranslation2d.rotateBy(robotYaw);
        intakeTranslation2d = intakeTranslation2d.plus(swerveDrive.getPose().getTranslation());

        Translation3d intakeTranslation3d = new Translation3d(
                                                    intakeTranslation2d.getX(), 
                                                    intakeTranslation2d.getY(), 
                                                    Constants.IntakeConstants.KINTAKE_TRANSLATION3D.getZ());

        Translation2d noteTranslation2d = noteDetector.getNoteFieldRelativePose();
        if (noteDetector.hasTargets()/* &&   noteTranslation2d.getDistance(swerveDrive.getPose().getTranslation()) < 1.2*/) {
            
            ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
            desiredSpeeds.vxMetersPerSecond = 
                m_translationalController.calculate(intakeTranslation3d.getX(), noteTranslation2d.getX());
            desiredSpeeds.vyMetersPerSecond = 
                m_translationalController.calculate(intakeTranslation3d.getY(), noteTranslation2d.getY());
            
            double scaler = Math.sqrt(Math.pow(drivingSpeedMps,2)/(Math.pow(desiredSpeeds.vxMetersPerSecond,2) + Math.pow(desiredSpeeds.vyMetersPerSecond,2)));
            desiredSpeeds.vxMetersPerSecond *= scaler;
            desiredSpeeds.vyMetersPerSecond *= scaler;
            
            double desiredRotation = Math.atan2(
                        swerveDrive.getPose().getY() - noteTranslation2d.getY(), 
                        swerveDrive.getPose().getX() - noteTranslation2d.getX());

            initialYaw = new Rotation2d(desiredRotation);

            desiredSpeeds.omegaRadiansPerSecond = // TODO: FOR SOME REASON, SIM inversion is DIFFRENT than IRL version
                m_rotationalController.calculate(
                    robotYaw.getRadians(), desiredRotation);

            
            
            Logger.recordOutput("IntakeNoteCommand/desiredRotationRad", desiredRotation);
            Logger.recordOutput("IntakeNoteCommand/desiredSpeeds", desiredSpeeds);

            swerveDrive.driveFieldRelative(desiredSpeeds);
        }
        else {
            ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0, 0, 0);

            if (over) {
                double goal = initialYaw.getRadians() - Math.toRadians(30);
                m_rotationalController.setGoal(goal);
                if (m_rotationalController.atGoal()) {
                    over = false;
                }
                else {
                    desiredSpeeds.omegaRadiansPerSecond = desiredSpeeds.omegaRadiansPerSecond = 
                    m_rotationalController.calculate(robotYaw.getRadians());
                    Logger.recordOutput("IntakeNoteCommand/desiredRotationRad",goal);
                }

            }
            else {
                double goal = initialYaw.getRadians() + Math.toRadians(30);
                m_rotationalController.setGoal(goal);
                if (m_rotationalController.atGoal()) {
                    over = true;
                }
                else {
                    desiredSpeeds.omegaRadiansPerSecond = desiredSpeeds.omegaRadiansPerSecond = 
                    m_rotationalController.calculate(robotYaw.getRadians());
                    Logger.recordOutput("IntakeNoteCommand/desiredRotationRad", goal);
                }
            }
            
            swerveDrive.driveFieldRelative(desiredSpeeds);
        }
        
    }

    @Override
    public boolean isFinished() {
        return intake.inIntake();
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("IntakeNoteCommand/driveInterrupted", interrupted);
        Logger.recordOutput("IntakeNoteCommand/end", true);
        swerveDrive.driveFieldRelative(new ChassisSpeeds(0, 0, 0));
    }
}

