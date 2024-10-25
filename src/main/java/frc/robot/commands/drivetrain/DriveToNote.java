package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.util.RebelUtil;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.drivetrain.vision.NoteDetector;
import frc.robot.subsystems.intakeComp.Intake;

// Command for driving the robot towards a detected note
public class DriveToNote extends Command {
    private final Intake intake; // Reference to the intake subsystem
    private final NoteDetector noteDetector; // Reference to the note detector subsystem
    private final SwerveDrive swerveDrive; // Reference to the swerve drive subsystem

    // PID controllers for translational and rotational control
    private final PIDController m_translationalController; 
    private final ProfiledPIDController m_rotationalController;

    private Rotation2d initialYaw = new Rotation2d();
    private boolean over = false;
    private double drivingSpeedMps = 1;
    private boolean initalReset = false;
    private Translation2d intialNotePose = null;

    public static boolean isRunning = false;

    // Constructor for the DriveToNote command
    public DriveToNote(SwerveDrive swerveDrive, Intake intakeSubsystem, NoteDetector noteDetector) {
        this.intake = intakeSubsystem; // Initialize intake reference
        this.noteDetector = noteDetector; // Initialize note detector reference
        this.swerveDrive = swerveDrive; // Initialize swerve drive reference

        // Configure PID controllers based on current mode (SIM or REAL)
        switch (Constants.currentMode) {
            case SIM:
                m_translationalController = new PIDController(1.2, 0, 0); // Simulated translational controller gains
                m_rotationalController = new ProfiledPIDController(2, 0, 0,
                 new Constraints(Constants.Auton.MAX_ANGULAR_VELO_RPS * 2 * Math.PI, 
                 Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED * 2 * Math.PI)); // Simulated rotational controller
                break;
        
            default:
                m_translationalController = new PIDController(1.6, 0, 0);
                m_rotationalController = new ProfiledPIDController(2, 0, 0.1,
                 new Constraints(Constants.Auton.MAX_ANGULAR_VELO_RPS * 2 * Math.PI, 
                 Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED * 2 * Math.PI)); // Real rotational controller
                
                m_rotationalController.setTolerance(Math.toRadians(5)); // Set tolerance for rotational controller
                break;
        }
        m_rotationalController.enableContinuousInput(0, Math.PI * 2); // Enable continuous input for rotation
        // addRequirements(swerveDrive); // Declare that this command requires the swerve drive subsystem
    }

    // Called when the command is initially scheduled
    @Override
    public void initialize() {
        intialNotePose = null;
        initalReset = false;
        initialYaw = swerveDrive.getPose().getRotation();
        Logger.recordOutput("IntakeNoteCommand/initialYaw", initialYaw);
        Logger.recordOutput("IntakeNoteCommand/end", false);

        double currentSpeed = Math.sqrt(Math.pow(swerveDrive.getMeasuredFeildRelativeSpeeds().vxMetersPerSecond,2) + 
                                        Math.pow(swerveDrive.getMeasuredFeildRelativeSpeeds().vyMetersPerSecond,2));

        drivingSpeedMps = RebelUtil.constrain(currentSpeed, 1, 1.6);
        Logger.recordOutput("IntakeNoteCommand/drivingSpeedMps", drivingSpeedMps);
    }

    // Called repeatedly while the command is scheduled
    @Override
    public void execute() {
        Rotation2d robotYaw = swerveDrive.getPose().getRotation(); // Get current robot yaw
        Translation2d intakeTranslation2d = Constants.IntakeConstants.KINTAKE_TRANSLATION3D.toTranslation2d(); 
        intakeTranslation2d = intakeTranslation2d.rotateBy(robotYaw); // Rotate intake translation by robot's yaw
        intakeTranslation2d = intakeTranslation2d.plus(swerveDrive.getPose().getTranslation()); // Add robot's translation

        Translation3d intakeTranslation3d = new Translation3d(
                                                    intakeTranslation2d.getX(), 
                                                    intakeTranslation2d.getY(), 
                                                    Constants.IntakeConstants.KINTAKE_TRANSLATION3D.getZ()); // Create a 3D translation for the intake

        Translation2d noteTranslation2d = noteDetector.getNoteFieldRelativePose(); // Get position of detected note
        if (noteDetector.hasTargets() && intialNotePose == null) {
            intialNotePose = noteTranslation2d;
        }

        if (intialNotePose != null && noteDetector.getNoteFieldRelativePose().getDistance(intialNotePose) <= 0.6 &&
            (noteDetector.hasTargets() || (noteDetector.getNoteRobotRelativePose().getX() <= 1.1 && noteDetector.getNoteRobotRelativePose().getX() > .6))) { // Check if any targets are detected
            
            isRunning = true;

            initalReset = false;
            Logger.recordOutput("distNoteFromInital", noteDetector.getNoteFieldRelativePose().getDistance(intialNotePose));
            //penis

            ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0,0,0); // Initialize desired speeds object
            
            // Calculate desired translational speeds using PID controller
            desiredSpeeds.vxMetersPerSecond = 
                m_translationalController.calculate(intakeTranslation3d.getX(), noteTranslation2d.getX());
            desiredSpeeds.vyMetersPerSecond = 
                m_translationalController.calculate(intakeTranslation3d.getY(), noteTranslation2d.getY());
            
            desiredSpeeds = RebelUtil.scaleSpeeds(drivingSpeedMps, desiredSpeeds);
            
            double desiredRotation = Math.atan2(
                        swerveDrive.getPose().getY() - noteTranslation2d.getY(), 
                        swerveDrive.getPose().getX() - noteTranslation2d.getX()); // Calculate desired rotation towards the note

            initialYaw = new Rotation2d(desiredRotation); // Update initial yaw to desired rotation

            desiredSpeeds.omegaRadiansPerSecond =
                m_rotationalController.calculate(
                    robotYaw.getRadians(), initialYaw.getRadians()); // Calculate rotational speed using PID controller
            
            Logger.recordOutput("IntakeNoteCommand/desiredRotationRad", desiredRotation); // Log desired rotation in radians
            Logger.recordOutput("IntakeNoteCommand/desiredSpeeds", desiredSpeeds); // Log desired speeds

            swerveDrive.driveFieldRelative(desiredSpeeds); // Command swerve drive to move with calculated speeds
        }
        else if (DriverStation.isAutonomous()) { 
            isRunning = true;
            
            if (!initalReset) {
                initialYaw = swerveDrive.getPose().getRotation();
                initalReset = true;
            }
            ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0, 0, 0); // Stop if no targets are detected

            if (over) { // If over flag is true, rotate back by -30 degrees from initial yaw
                double goal = initialYaw.getRadians() - Math.toRadians(30);
                m_rotationalController.setGoal(goal); // Set goal for rotational controller
                
                if (m_rotationalController.atGoal()) { 
                    over = false; // Toggle over flag if goal is reached
                }
                else {
                    desiredSpeeds.omegaRadiansPerSecond = 
                    m_rotationalController.calculate(robotYaw.getRadians()); // Calculate rotational speed towards goal
                    Logger.recordOutput("IntakeNoteCommand/desiredRotationRad",goal); // Log goal rotation in radians
                }
                swerveDrive.driveFieldRelative(desiredSpeeds); // Command swerve drive to move with calculated speeds
            }
            else { // If over flag is false, rotate forward by +30 degrees from initial yaw
                double goal = initialYaw.getRadians() + Math.toRadians(30);
                m_rotationalController.setGoal(goal); // Set goal for rotational controller
                
                if (m_rotationalController.atGoal()) {
                    over = true; // Toggle over flag if goal is reached
                }
                desiredSpeeds.omegaRadiansPerSecond =
                m_rotationalController.calculate(robotYaw.getRadians()); // Calculate rotational speed towards goal
                Logger.recordOutput("IntakeNoteCommand/desiredRotationRad", goal); // Log goal rotation in radians
                swerveDrive.driveFieldRelative(desiredSpeeds); // Command swerve drive to move with calculated speeds while rotating
            }
            
        }
        else {
            isRunning = false;
        }
        
    }

    @Override
    public boolean isFinished() {
        return intake.inIntake(); // Command finishes when the intake system indicates it has successfully ingested a note
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("IntakeNoteCommand/driveInterrupted", interrupted); // Log whether command was interrupted or finished normally
        Logger.recordOutput("IntakeNoteCommand/end", true); // Log command end status
        // swerveDrive.driveFieldRelative(new ChassisSpeeds(0, 0, 0)); // Stop all movement of the swerve drive at command end
        isRunning = false;
        if (!interrupted) {
            swerveDrive.driveFieldRelative(new ChassisSpeeds(0,0,0));
        }
        
    }
}
