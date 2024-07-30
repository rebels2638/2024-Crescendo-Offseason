// package frc.robot.commands.compositions;

// import java.io.SequenceInputStream;
// import java.net.InetSocketAddress;

// import org.littletonrobotics.junction.Logger;

// import com.pathplanner.lib.path.PathPlannerTrajectory.State;

// import edu.wpi.first.math.controller.HolonomicDriveController;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants;
// import frc.robot.commands.intake.InIntake;
// import frc.robot.commands.intake.OutIntake;
// import frc.robot.commands.intake.RollIntakeIn;
// import frc.robot.commands.intake.RollIntakeInSlow;
// import frc.robot.commands.intake.RollIntakeOut;
// import frc.robot.commands.intake.StopIntake;
// import frc.robot.commands.pivot.PivotToTorus;
// import frc.robot.commands.pivot.PivotTurtle;
// import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
// import frc.robot.subsystems.drivetrain.vision.NoteDetector;
// import frc.robot.subsystems.intakeComp.Intake;
// import frc.robot.subsystems.pivotComp.Pivot;

// public class DriveToNote extends Command {
//     private final Intake intake;
//     private final NoteDetector noteDetector;
//     private final SwerveDrive swerveDrive;
//     private Translation2d notePose = new Translation2d();

//     private final PIDController m_translationalController;
//     private final ProfiledPIDController m_rotationalController;
//     // private final HolonomicDriveController controller;

//     private final Pivot pivot;
//     private Rotation2d initialYaw = new Rotation2d();
//     private boolean over = false;

//     private SequentialCommandGroup startGroup;

//     public DriveToNote(SwerveDrive swerveDrive, Intake intakeSubsystem, NoteDetector noteDetector, Pivot pivot) {
//         this.intake = intakeSubsystem;
//         this.noteDetector = noteDetector;
//         this.swerveDrive = swerveDrive;
//         this.pivot = pivot;

//         switch (Constants.currentMode) {
//             case SIM:
//                 m_translationalController = new PIDController(1.2, 0, 0);
//                 m_rotationalController = new ProfiledPIDController(1.2, 0, 0,
//                  new Constraints(Constants.Auton.MAX_ANGULAR_VELO_RPS * 2 * Math.PI, 
//                  Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED * 2 * Math.PI));
//                 break;
        
//             default:
//                 m_translationalController = new PIDController(1, 0, 0);
//                 m_rotationalController = new ProfiledPIDController(1, 0, 0,
//                  new Constraints(Constants.Auton.MAX_ANGULAR_VELO_RPS * 2 * Math.PI, 
//                  Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED * 2 * Math.PI));
                
//                 m_rotationalController.setTolerance(Math.toRadians(5));
//                 break;
//         }
//         // controller = new HolonomicDriveController(m_translationalController, m_translationalController, m_rotationalController);

//         addRequirements(swerveDrive);
//     }

//     @Override
//     public void initialize() {
//         startGroup = new SequentialCommandGroup(
//             new PivotToTorus(),
//             new RollIntakeIn()
//         );
//         startGroup.schedule();
//         notePose = noteDetector.getNoteFeildRelativePose();
//         initialYaw = swerveDrive.getPose().getRotation();
//     }

//     @Override
//     public void execute() {
//         Rotation2d robotYaw = swerveDrive.getPose().getRotation();
//         Translation2d intakeTranslation2d = Constants.IntakeConstants.KINTAKE_TRANSLATION3D.toTranslation2d();
//         intakeTranslation2d = intakeTranslation2d.rotateBy(robotYaw);
//         intakeTranslation2d = intakeTranslation2d.plus(swerveDrive.getPose().getTranslation());

//         Translation3d intakeTranslation3d = new Translation3d(
//                                                     intakeTranslation2d.getX(), 
//                                                     intakeTranslation2d.getY(), 
//                                                     Constants.IntakeConstants.KINTAKE_TRANSLATION3D.getZ());

//         Logger.recordOutput("IntakeNoteCommand/notePresent", noteDetector.notePresent());
//         if (noteDetector.notePresent()) {
//             ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
//             desiredSpeeds.vxMetersPerSecond = 
//                 m_translationalController.calculate(intakeTranslation3d.getX(), noteDetector.getNoteFeildRelativePose().getX());
//             desiredSpeeds.vyMetersPerSecond = 
//                 m_translationalController.calculate(intakeTranslation3d.getY(), noteDetector.getNoteFeildRelativePose().getY());

//             double desiredRotation = Math.atan2(
//                         noteDetector.getNoteFeildRelativePose().getY() - swerveDrive.getPose().getY(), 
//                         noteDetector.getNoteFeildRelativePose().getX() - swerveDrive.getPose().getX());

//             initialYaw = new Rotation2d(desiredRotation);

//             desiredSpeeds.omegaRadiansPerSecond = 
//                 m_rotationalController.calculate(
//                     robotYaw.getRadians(), desiredRotation);

            
            
//             Logger.recordOutput("IntakeNoteCommand/desiredRotationRad", desiredRotation);
//             Logger.recordOutput("IntakeNoteCommand/desiredSpeeds", desiredSpeeds);

//             swerveDrive.driveFieldRelative(desiredSpeeds);
//         }
//         else {
//             ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0, 0, 0);

//             if (over) {
//                 double goal = initialYaw.getRadians() - Math.toRadians(30);
//                 m_rotationalController.setGoal(goal);
//                 if (m_rotationalController.atGoal()) {
//                     over = false;
//                 }
//                 else {
//                     desiredSpeeds.omegaRadiansPerSecond = desiredSpeeds.omegaRadiansPerSecond = 
//                     m_rotationalController.calculate(robotYaw.getRadians());
//                     Logger.recordOutput("IntakeNoteCommand/desiredRotationRad",goal);
//                 }

//             }
//             else {
//                 double goal = initialYaw.getRadians() + Math.toRadians(30);
//                 m_rotationalController.setGoal(goal);
//                 if (m_rotationalController.atGoal()) {
//                     over = true;
//                 }
//                 else {
//                     desiredSpeeds.omegaRadiansPerSecond = desiredSpeeds.omegaRadiansPerSecond = 
//                     m_rotationalController.calculate(robotYaw.getRadians());
//                     Logger.recordOutput("IntakeNoteCommand/desiredRotationRad", goal);
//                 }
//             }
            
//             swerveDrive.driveFieldRelative(desiredSpeeds);
//         }
        
//     }

//     @Override
//     public boolean isFinished() {
//         return intake.inIntake();
//     }

//     @Override
//     public void end(boolean interrupted) {
        
//         Logger.recordOutput("IntakeNoteCommand/interrupted", interrupted);

//         SequentialCommandGroup endGroup = new SequentialCommandGroup(
//             new StopIntake(),
//             new PivotTurtle(),
//             new RollIntakeOut(), 
//             new WaitCommand(0.15),
//             new OutIntake(),
//             new StopIntake(), 
//             new RollIntakeInSlow(),
//             new InIntake(),
//             new WaitCommand(0.1),
//             new StopIntake()
//         );

//         if (interrupted) {
//             Command stop = new CancelIntakeNote(endGroup, null);
//             stop.schedule();
//             return;
//         }

//         endGroup.schedule();
//     }
// }

