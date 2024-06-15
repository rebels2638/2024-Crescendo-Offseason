package frc.robot.subsystems.drivetrain.swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0,0,0);

    private final GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private Rotation2d yaw = new Rotation2d();

    private SwerveDrivePoseEstimator m_poseEstimator;

    private final Notifier odometryThread;
    private final Lock odometryLock = new ReentrantLock();

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
                    new ModuleIOSIM(),
                    new ModuleIOSIM(),
                    new ModuleIOSIM(),
                    new ModuleIOSIM()
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
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, .5),
            VecBuilder.fill(0.5, 0.5, .1));

        odometryThread = new Notifier(this::updateOdometry);
    }

    private void updateInputs() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("SwerveDrive/Gyro", gyroInputs);

        prevTime = Timer.getFPGATimestamp();

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
        }

    }

    @Override 
    public void periodic() {
        odometryLock.lock();
        updateInputs();
        odometryLock.unlock();

        Logger.recordOutput("SwerveDrive/estimatedPose", m_poseEstimator.getEstimatedPosition());

        Logger.recordOutput("SwerveDrive/desiredSpeeds", desiredSpeeds);

        SwerveModuleState[] desiredModuleStates = m_kinematics.toSwerveModuleStates(desiredSpeeds);
        for (int i = 0; i < 4; i++) {
            // desiredModuleStates[i] = SwerveModuleState.optimize(desiredModuleStates[i], measuredModuleStates[i].angle);
            modules[i].setState(desiredModuleStates[i]);
        }
        Logger.recordOutput("SwerveDrive/desiredModuleStates", desiredModuleStates);

    }

    public void driveFeildRelative(ChassisSpeeds speeds) {
        desiredSpeeds = speeds;
    }

    public void driveRobotRelatve(ChassisSpeeds speeds) {
        desiredSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, gyroInputs.yaw);
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

    private void updateOdometry() {
        odometryLock.lock();
        updateInputs();
        m_poseEstimator.update(yaw, new SwerveModulePosition[] {
            new SwerveModulePosition(moduleInputs[0].drivePositionMeters, new Rotation2d(moduleInputs[0].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[1].drivePositionMeters, new Rotation2d(moduleInputs[1].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[2].drivePositionMeters, new Rotation2d(moduleInputs[2].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[3].drivePositionMeters, new Rotation2d(moduleInputs[3].anglePositionRad))
        });
        odometryLock.unlock();
    } 
}
