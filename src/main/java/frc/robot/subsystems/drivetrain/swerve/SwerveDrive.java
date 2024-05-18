package frc.robot.subsystems.drivetrain.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase{

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        Constants.DrivetrainConstants.frontLeftPositionMeters,
        Constants.DrivetrainConstants.frontRightPositionMeters,
        Constants.DrivetrainConstants.backLeftPositionMeters,
        Constants.DrivetrainConstants.backRightPositionMeters);

    private Module[] modules;
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0,0,0);

    private final GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    public SwerveDrive(Module[] modules, GyroIO gyroIO) {
        this.modules = modules;
        this.gyroIO = gyroIO;
    }

    @Override 
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Gyro", gyroInputs);

        SwerveModuleState[] desiredModuleStates = m_kinematics.toSwerveModuleStates(desiredSpeeds);
        for (int i = 0; i < 4; i++) {
            modules[i].setModuleState(desiredModuleStates[i]);
        }
    }

    public void driveFeildRelative(ChassisSpeeds speeds) {
        desiredSpeeds = speeds;
    }

    public void driveRobotRelatve(ChassisSpeeds speeds) {
        desiredSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, gyroInputs.yaw);
    }
}
