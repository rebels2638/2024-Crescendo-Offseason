package frc.robot.subsystems.drivetrain.swerve;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SwerveDrive extends SubsystemBase{

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        Constants.DrivetrainConstants.kFRONT_LEFT_POSITION_METERS,
        Constants.DrivetrainConstants.kFRONT_RIGHT_POSITION_METERS,
        Constants.DrivetrainConstants.kBACK_LEFT_POSITION_METERS,
        Constants.DrivetrainConstants.kBACK_RIGHT_POSITION_METERS);

    private ModuleIO[] modules;
    private ModuleIOInputsAutoLogged[] moduleInputs = {
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged()
    };

    private ChassisSpeeds desiredRobotRelativeSpeeds = new ChassisSpeeds(0,0,0);

    private final GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private Rotation2d yaw = new Rotation2d(0);

    private SwerveDrivePoseEstimator m_poseEstimator;

    private final Notifier odometryThread;
    private final Lock odometryLock = new ReentrantLock();

    private ChassisSpeeds measuredRobotRelativeSpeeds = new ChassisSpeeds(0,0,0);

    private SwerveModuleState[] measuredModuleStates = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private double prevTime = Timer.getFPGATimestamp();
    
    public SwerveDrive() {
        switch (Constants.currentMode) {
            default:
                modules = new ModuleIO[] {
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                };
                gyroIO = new GyroIO() {};
                break;
        }

        m_poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d())
            },
            new Pose2d());

        odometryThread = new Notifier(this::updateOdometry);
    }

    private void updateInputs() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("SwerveDrive/Gyro", gyroInputs);


        for (int i = 0; i < 4; i++) {
            modules[i].updateInputs(moduleInputs[i]);
            Logger.processInputs("SwerveDrive/Module" + i, moduleInputs[i]);

            measuredModuleStates[i].angle = new Rotation2d(moduleInputs[i].anglePositionRad);
            measuredModuleStates[i].speedMetersPerSecond = moduleInputs[i].driveVelocityMps;
        }
        Logger.recordOutput("SwerveDrive/measuredModuleStates", measuredModuleStates);

        if (gyroInputs.isConnected) {
            yaw = gyroInputs.yaw;
        }
        else {
            yaw = new Rotation2d(yaw.getRadians() + 
                                m_kinematics.toChassisSpeeds(
                                measuredModuleStates[0],
                                measuredModuleStates[1], 
                                measuredModuleStates[2], 
                                measuredModuleStates[3]).omegaRadiansPerSecond * (Timer.getFPGATimestamp() - prevTime));

            yaw = new Rotation2d(yaw.getRadians() % (2 * Math.PI));
        }

        prevTime = Timer.getFPGATimestamp();
    }

    @Override 
    public void periodic() {
        odometryLock.lock();
        updateInputs();
        odometryLock.unlock();

        // if (DriverStation.isAutonomous()) {
        //     // type PathPlannerAuto
        //     String name = RobotContainer.getInstance().getSelectedAuto();
        //     // PathPlannerAuto auto = (PathPlannerAuto) RobotContainer.getInstance().getAutonomousCommand();
        //     // auto.initialize();
        //     // List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(name);
            
        //     Command auto = AutoBuilder.buildAuto(name);
        //     CommandScheduler.getInstance().
           
        // }

        m_poseEstimator.update(yaw, new SwerveModulePosition[] {
            new SwerveModulePosition(moduleInputs[0].drivePositionMeters, new Rotation2d(moduleInputs[0].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[1].drivePositionMeters, new Rotation2d(moduleInputs[1].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[2].drivePositionMeters, new Rotation2d(moduleInputs[2].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[3].drivePositionMeters, new Rotation2d(moduleInputs[3].anglePositionRad))
        });

        measuredRobotRelativeSpeeds = m_kinematics.toChassisSpeeds(
            measuredModuleStates[0],
            measuredModuleStates[1],
            measuredModuleStates[2],
            measuredModuleStates[3]);
        
        Logger.recordOutput("SwerveDrive/estimYaw", yaw.getRadians());
        Logger.recordOutput("SwerveDrive/estimatedPose", new double[] {
            m_poseEstimator.getEstimatedPosition().getTranslation().getX(),
            m_poseEstimator.getEstimatedPosition().getTranslation().getY(),
            m_poseEstimator.getEstimatedPosition().getRotation().getRadians()
        });

        Logger.recordOutput("SwerveDrive/desiredRobotRelativeSpeeds", desiredRobotRelativeSpeeds);
        Logger.recordOutput("SwerveDrive/measuredRobotRelativeSpeeds", measuredRobotRelativeSpeeds);

        SwerveModuleState[] desiredModuleStates = m_kinematics.toSwerveModuleStates(desiredRobotRelativeSpeeds);
        for (int i = 0; i < 4; i++) {
            desiredModuleStates[i] = SwerveModuleState.optimize(desiredModuleStates[i], measuredModuleStates[i].angle);
            modules[i].setState(desiredModuleStates[i], 0);
        }
        Logger.recordOutput("SwerveDrive/desiredModuleStates", desiredModuleStates);

    }

    public void driveFeildRelative(ChassisSpeeds speeds) {
        desiredRobotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, yaw);
    }

    public void driveRobotRelatve(ChassisSpeeds speeds) {
        desiredRobotRelativeSpeeds = speeds;
    }

    public void resetPose(Pose2d pose) {
        odometryLock.lock();
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modules[i].resetPosition();
            positions[i] = new SwerveModulePosition(0.0, new Rotation2d(moduleInputs[i].anglePositionRad));
        }

        gyroIO.reset(new Rotation3d(0,0, pose.getRotation().getRadians()));
        m_poseEstimator.resetPosition(pose.getRotation(), positions, pose);
        odometryLock.unlock();
    }

    public ChassisSpeeds getMeasuredRobotRelativeSpeeds() {
        return measuredRobotRelativeSpeeds;
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    private void updateOdometry() {
        odometryLock.lock();
        // updateInputs();
        // m_poseEstimator.update(yaw, new SwerveModulePosition[] {
        //     new SwerveModulePosition(moduleInputs[0].drivePositionMeters, new Rotation2d(moduleInputs[0].anglePositionRad)),
        //     new SwerveModulePosition(moduleInputs[1].drivePositionMeters, new Rotation2d(moduleInputs[1].anglePositionRad)),
        //     new SwerveModulePosition(moduleInputs[2].drivePositionMeters, new Rotation2d(moduleInputs[2].anglePositionRad)),
        //     new SwerveModulePosition(moduleInputs[3].drivePositionMeters, new Rotation2d(moduleInputs[3].anglePositionRad))
        // });
        odometryLock.unlock();
    } 
}
