package frc.robot.subsystems.drivetrain.swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {

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

    private PIDController m_angleFeedbackController = new PIDController(1, 0, 0);;
    private DriveFFController driveFFController = new DriveFFController();

    private PIDController m_translationalFeedbackController = new PIDController(1, 0, 0);

    private SwerveModuleState[] measuredModuleStates = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private SwerveModuleState[] previousDesiredStates = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private double prevTime = Timer.getFPGATimestamp();

    private final Config m_sysDriveConfig;
    private final SysIdRoutine m_sysDriveIdRoutine;
    
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

                m_sysDriveConfig = new Config(Units.Volts.per(Units.Second).of(1), Units.Volts.of(7), Units.Second.of(10));
                
                break;
            default:
                modules = new ModuleIO[] {
                    new ModuleIOTalon(0),
                    new ModuleIOTalon(1),
                    new ModuleIOTalon(2),
                    new ModuleIOTalon(3)
                };
                gyroIO = new GyroIONavX();

                m_sysDriveConfig = new Config(Units.Volts.per(Units.Second).of(1), Units.Volts.of(7), Units.Second.of(10));

                break;
        }

        Consumer<Measure<Voltage>> driveConsumer = volts -> {
            for (int i = 0; i < 4; i++) {
                modules[i].setDriveVoltage(volts.baseUnitMagnitude());
            }};
        Consumer<SysIdRoutineLog> logConsumer = log -> {
            for (int i = 0; i < 4; i++) {
                log.motor("module" + i).
                    voltage(Units.Volts.of(moduleInputs[i].driveVoltage)).
                    linearVelocity(Units.MetersPerSecond.of(moduleInputs[i].driveVelocityMps)).
                    linearPosition(Units.Meters.of(moduleInputs[i].drivePositionMeters));
            }};
        Mechanism m_driveMechanism = new Mechanism(driveConsumer, logConsumer, this);
        m_sysDriveIdRoutine = new SysIdRoutine(m_sysDriveConfig, m_driveMechanism);

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

        }

        prevTime = Timer.getFPGATimestamp();
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

        desiredRobotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredFeildRelativeSpeeds, yaw);
        Logger.recordOutput("SwerveDrive/desiredRobotRelativeSpeeds", desiredRobotRelativeSpeeds);
        Logger.recordOutput("SwerveDrive/measuredRobotRelativeSpeeds", measuredRobotRelativeSpeeds);
        Logger.recordOutput("SwerveDrive/desiredFeildRelativeSpeeds", desiredFeildRelativeSpeeds);
        Logger.recordOutput("SwerveDrive/measuredFeildRelativeSpeeds", measuredFeildRelativeSpeeds);


        
        double estimatedVyDriftMetersPerSecond = driveFFController.calculate(desiredFeildRelativeSpeeds.vxMetersPerSecond, desiredFeildRelativeSpeeds.omegaRadiansPerSecond);
        double estimatedVxDriftMetersPerSecond = driveFFController.calculate(desiredFeildRelativeSpeeds.vyMetersPerSecond, desiredFeildRelativeSpeeds.omegaRadiansPerSecond);
        Logger.recordOutput("SwerveDrive/estimatedVxDriftMetersPerSecond", estimatedVxDriftMetersPerSecond);
        Logger.recordOutput("SwerveDrive/estimatedVyDriftMetersPerSecond", estimatedVyDriftMetersPerSecond);

        ChassisSpeeds correctedSpeeds = desiredFeildRelativeSpeeds;
        correctedSpeeds.vxMetersPerSecond = correctedSpeeds.vxMetersPerSecond + estimatedVxDriftMetersPerSecond + 
                                            m_translationalFeedbackController.calculate(measuredFeildRelativeSpeeds.vxMetersPerSecond, 
                                            desiredFeildRelativeSpeeds.vxMetersPerSecond);
        correctedSpeeds.vyMetersPerSecond = correctedSpeeds.vyMetersPerSecond - estimatedVyDriftMetersPerSecond + 
                                            m_translationalFeedbackController.calculate(measuredFeildRelativeSpeeds.vyMetersPerSecond, 
                                            desiredFeildRelativeSpeeds.vyMetersPerSecond);
        correctedSpeeds.omegaRadiansPerSecond = correctedSpeeds.omegaRadiansPerSecond + m_angleFeedbackController.calculate(measuredFeildRelativeSpeeds.omegaRadiansPerSecond, desiredFeildRelativeSpeeds.omegaRadiansPerSecond);
        correctedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(correctedSpeeds, yaw);

        Logger.recordOutput("SwerveDrive/correctedSpeeds", correctedSpeeds);

        SwerveModuleState[] desiredModuleStates = m_kinematics.toSwerveModuleStates(correctedSpeeds);
        // if (Math.abs(desiredRobotRelativeSpeeds.vxMetersPerSecond) <= 0.01 && 
        //     Math.abs(desiredRobotRelativeSpeeds.vyMetersPerSecond) <= 0.01 && 
        //     Math.abs(desiredRobotRelativeSpeeds.omegaRadiansPerSecond) <= 0.1) {
        //     desiredModuleStates = measuredModuleStates;
        //     for (int i = 0; i < 4; i++) {
        //         desiredModuleStates[i].speedMetersPerSecond = 0;
        //     }
        // } 
        for (int i = 0; i < 4; i++) {
            Logger.recordOutput("SwerveDrive/unoptimizedDesiredModuleStates", desiredModuleStates);

            desiredModuleStates[i] = SwerveModuleState.optimize(desiredModuleStates[i], measuredModuleStates[i].angle);
            modules[i].setState(desiredModuleStates[i]);
        }

        Logger.recordOutput("SwerveDrive/desiredModuleStates", desiredModuleStates);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        desiredFeildRelativeSpeeds = speeds;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        desiredFeildRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, yaw);
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

    public void zeroGyro() {
        gyroIO.zero();
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

    public Command sysIDriveQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysDriveIdRoutine.quasistatic(direction);
    }

    public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
        return m_sysDriveIdRoutine.dynamic(direction);
    }
}