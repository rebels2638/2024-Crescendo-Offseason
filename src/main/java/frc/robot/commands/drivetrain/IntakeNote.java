package frc.robot.commands.drivetrain;

import java.net.InetSocketAddress;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.intake.RollIntakeIn;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.drivetrain.vision.NoteDetector;
import frc.robot.subsystems.intake.Intake;

public class IntakeNote extends Command {
    private final Intake intake;
    private final NoteDetector noteDetector;
    private final SwerveDrive swerveDrive;
    private Translation3d notePose = new Translation3d();

    private final PIDController m_translationalController;
    private final ProfiledPIDController m_rotationalController;
    // private final HolonomicDriveController controller;

    private Rotation2d initialYaw = new Rotation2d();

    public IntakeNote(SwerveDrive swerveDrive, Intake intakeSubsystem, NoteDetector noteDetector) {
        this.intake = intakeSubsystem;
        this.noteDetector = noteDetector;
        this.swerveDrive = swerveDrive;

        switch (Constants.currentMode) {
            case SIM:
                m_translationalController = new PIDController(1, 0, 0);
                m_rotationalController = new ProfiledPIDController(1, 0, 0,
                 new Constraints(Constants.Auton.MAX_ANGULAR_VELO_RPS * 2 * Math.PI, 
                 Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED * 2 * Math.PI));
                break;
        
            default:
                m_translationalController = new PIDController(1, 0, 0);
                m_rotationalController = new ProfiledPIDController(1, 0, 0,
                 new Constraints(Constants.Auton.MAX_ANGULAR_VELO_RPS * 2 * Math.PI, 
                 Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED * 2 * Math.PI));
                
                m_rotationalController.setTolerance(Math.toRadians(5));
                break;
        }
        // controller = new HolonomicDriveController(m_translationalController, m_translationalController, m_rotationalController);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        Command intakeCommand = new RollIntakeIn(intake);
        intakeCommand.schedule();
        notePose = noteDetector.getNoteFeildRelativePose();
        initialYaw = swerveDrive.getPose().getRotation();
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

        Logger.recordOutput("IntakeNoteCommand/notePresent", noteDetector.notePresent());
        if (noteDetector.notePresent()) {
            ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
            desiredSpeeds.vxMetersPerSecond = 
                m_translationalController.calculate(intakeTranslation3d.getX(), noteDetector.getNoteFeildRelativePose().getX());
            desiredSpeeds.vyMetersPerSecond = 
                m_translationalController.calculate(intakeTranslation3d.getY(), noteDetector.getNoteFeildRelativePose().getY());

            double desiredRotation = Math.atan2(
                        noteDetector.getNoteFeildRelativePose().getY() - swerveDrive.getPose().getY(), 
                        noteDetector.getNoteFeildRelativePose().getX() - swerveDrive.getPose().getX());

            desiredSpeeds.omegaRadiansPerSecond = 
                m_rotationalController.calculate(
                    robotYaw.getRadians(), desiredRotation);

            
            
            Logger.recordOutput("IntakeNoteCommand/desiredRotationRad", desiredRotation);

            swerveDrive.driveFieldRelative(desiredSpeeds);
        }
        else {
            ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0, 0, 0);
            desiredSpeeds.omegaRadiansPerSecond = desiredSpeeds.omegaRadiansPerSecond = 
                m_rotationalController.calculate(
                    robotYaw.getRadians(), 
                    initialYaw.getRadians() + Math.toRadians(30)
                );
            Logger.recordOutput("IntakeNoteCommand/desiredRotationRad", initialYaw.getRadians() + Math.toRadians(30));

            if (m_rotationalController.atSetpoint()) {
                desiredSpeeds.omegaRadiansPerSecond = desiredSpeeds.omegaRadiansPerSecond = 
                m_rotationalController.calculate(
                    robotYaw.getRadians(), 
                    initialYaw.getRadians() - Math.toRadians(30)
                );
                Logger.recordOutput("IntakeNoteCommand/desiredRotationRad", initialYaw.getRadians() - Math.toRadians(30));

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
    }
}
