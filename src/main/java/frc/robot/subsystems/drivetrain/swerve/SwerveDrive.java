package frc.robot.subsystems.drivetrain.swerve;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    private ChassisSpeeds desiredRobotRelativeSpeeds = new ChassisSpeeds(0,0,0);
    private ChassisSpeeds desiredFeildRelativeSpeeds = new ChassisSpeeds(0,0,0);

    private final GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private int syncCount = 0;

    private Rotation2d yaw = new Rotation2d(0);

    private SwerveDrivePoseEstimator m_poseEstimator;

    private final Notifier odometryThread;
    private final Lock odometryLock = new ReentrantLock();

    private ChassisSpeeds measuredRobotRelativeSpeeds = new ChassisSpeeds(0,0,0);
    private ChassisSpeeds measuredFeildRelativeSpeeds = new ChassisSpeeds(0,0,0);

    private PIDController m_angleFeedbackController = new PIDController(0.00, 0.0, 0.000);
    private SimpleMotorFeedforward m_angleFeedForwardController = new SimpleMotorFeedforward(0, 1);

    private PIDController m_translationalFeedbackController = new PIDController(0.00, 0.0, 0.000);

    private SwerveModuleState[] measuredModuleStates = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private double prevTime = Timer.getFPGATimestamp();
    
    public SwerveDrive() {
        switch (Constants.currentMode) {
            case SIM:
                modules = new ModuleIO[] {
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                };
                gyroIO = new GyroIO() {};
                break;
            default:
                modules = new ModuleIO[] {
                    new ModuleIOTalon(0),
                    new ModuleIOTalon(1),
                    new ModuleIOTalon(2),
                    new ModuleIOTalon(3)
                };
                gyroIO = new GyroIONavX();
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
        odometryThread.startPeriodic(0.02);
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
                                measuredModuleStates[3]).omegaRadiansPerSecond * (Timer.getFPGATimestamp() - prevTime)); // TODO: discrepancy between prevTime being set and used in the calculation, causing a fixed offset

            yaw = new Rotation2d(yaw.getRadians() % (2 * Math.PI));

        prevTime = Timer.getFPGATimestamp();
        }

    }

    @Override 
    public void periodic() {
        odometryLock.lock();
        updateOdometry();
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

        measuredFeildRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(measuredRobotRelativeSpeeds, yaw);
        
        Logger.recordOutput("SwerveDrive/estimYaw", yaw.getRadians());
        Logger.recordOutput("SwerveDrive/estimatedPose", new double[] {
            m_poseEstimator.getEstimatedPosition().getTranslation().getX(),
            m_poseEstimator.getEstimatedPosition().getTranslation().getY(),
            m_poseEstimator.getEstimatedPosition().getRotation().getRadians()
        });

        Logger.recordOutput("SwerveDrive/desiredRobotRelativeSpeeds", desiredRobotRelativeSpeeds);
        Logger.recordOutput("SwerveDrive/measuredRobotRelativeSpeeds", measuredRobotRelativeSpeeds);
        Logger.recordOutput("SwerveDrive/desiredFeildRelativeSpeeds", desiredFeildRelativeSpeeds);
        Logger.recordOutput("SwerveDrive/measuredFeildRelativeSpeeds", measuredFeildRelativeSpeeds);


        ChassisSpeeds correctedSpeeds = desiredRobotRelativeSpeeds;
        correctedSpeeds.omegaRadiansPerSecond = 
            m_angleFeedForwardController.calculate(desiredRobotRelativeSpeeds.omegaRadiansPerSecond, Math.signum(desiredRobotRelativeSpeeds.omegaRadiansPerSecond - measuredRobotRelativeSpeeds.omegaRadiansPerSecond)) + 
            m_angleFeedbackController.calculate(measuredRobotRelativeSpeeds.omegaRadiansPerSecond, desiredRobotRelativeSpeeds.omegaRadiansPerSecond);

        Logger.recordOutput("SwerveDrive/correctedSpeeds", correctedSpeeds);

        SwerveModuleState[] desiredModuleStates = m_kinematics.toSwerveModuleStates(correctedSpeeds);
        for (int i = 0; i < 4; i++) {
            desiredModuleStates[i] = SwerveModuleState.optimize(desiredModuleStates[i], measuredModuleStates[i].angle);
            modules[i].setState(desiredModuleStates[i], 0);
        }

        Logger.recordOutput("SwerveDrive/desiredModuleStates", desiredModuleStates);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        desiredFeildRelativeSpeeds = speeds;
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
        odometryLock.lock();
        Pose2d estimate = m_poseEstimator.getEstimatedPosition();
        odometryLock.unlock();
        return estimate;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(moduleInputs[0].drivePositionMeters, new Rotation2d(moduleInputs[0].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[1].drivePositionMeters, new Rotation2d(moduleInputs[1].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[2].drivePositionMeters, new Rotation2d(moduleInputs[2].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[3].drivePositionMeters, new Rotation2d(moduleInputs[3].anglePositionRad))
        };
    }

    public void synchronizeModuleEncoders() {
        for (ModuleIO m : modules) {m.queueSynchronizeEncoders();}
    }

    private void updateOdometry() {
        odometryLock.lock();
        try {
            updateInputs();
            m_poseEstimator.update(yaw, getSwerveModulePositions());

            double velocitySum = 0.0;

            for (ModuleIOInputsAutoLogged m : moduleInputs) {
                velocitySum += Math.abs(m.driveVelocityMps);
            }

            if (velocitySum < 0.01 && ++syncCount > 5) {
                synchronizeModuleEncoders();
                syncCount = 0;
            }
        }

        catch (Exception e) {
            odometryLock.unlock();
            throw e;
        }

        odometryLock.unlock();
    }

    public void resetOdometry() {
        odometryLock.lock();
        m_poseEstimator.resetPosition(gyroInputs.yaw, getSwerveModulePositions(), getPose());
        odometryLock.unlock();
        m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, gyroInputs.yaw)); // TODO: this is yaw in radians right?
    }
}
