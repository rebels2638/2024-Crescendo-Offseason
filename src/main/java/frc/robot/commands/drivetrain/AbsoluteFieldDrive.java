package frc.robot.commands.drivetrain;


import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;


public class AbsoluteFieldDrive extends Command {
  private final SwerveDrive swerve;
  private final DoubleSupplier vX, vY, heading;

  public AbsoluteFieldDrive(SwerveDrive swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;

    addRequirements(swerve);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    ChassisSpeeds speeds = new ChassisSpeeds(
                          vX.getAsDouble() * Constants.DrivetrainConstants.kMAX_SPEED_METERS_PER_SECOND, 
                          vY.getAsDouble() * Constants.DrivetrainConstants.kMAX_SPEED_METERS_PER_SECOND, 
                          heading.getAsDouble() * Constants.DrivetrainConstants.kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND);
    swerve.driveFeildRelative(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}