package frc.robot.subsystems.drivetrain.swerve;

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

    public SwerveDrive() {
        switch (Constants.currentMode) {
            case REAL:
                for (int i = 0; i < 4; i++) {
                    modules[i] = new Module(new ModuleIOTalon(i));
                }
            default:
                break;
        }
        
    }

    @Override 
    public void periodic() {
        SwerveModuleState[] desiredModuleStates = m_kinematics.toSwerveModuleStates(desiredSpeeds);
        for (int i = 0; i < 4; i++) {
            modules[i].setModuleState(desiredModuleStates[i]);
        }
    }

    public void driveFeildRelative(ChassisSpeeds speeds) {
        desiredSpeeds = speeds;
    }

    public void driveRobotRelatve(ChassisSpeeds speeds) {
        
    }
}
