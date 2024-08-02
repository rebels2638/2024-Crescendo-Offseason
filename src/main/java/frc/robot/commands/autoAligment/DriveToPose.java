package frc.robot.commands.autoAligment;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

import com.pathplanner.lib.util.GeometryUtil;
 

public class DriveToPose extends Command {
    private Command followPathHolonomic;

    private Pose2d endGoal;

    public DriveToPose(Pose2d endGoal, SwerveDrive swerveDrive) {
        this.endGoal = endGoal;
    }

    // Builds a follow path holonomic path using field constants.
    @Override
    public void initialize() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            endGoal = GeometryUtil.flipFieldPose(endGoal);
        } 
        
        followPathHolonomic = AutoBuilder.pathfindToPose(endGoal, new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION, 
            Constants.Auton.MAX_ANGULAR_VELO_RPS, Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED), 0, 0);
        CommandScheduler.getInstance().schedule(followPathHolonomic);
    }

    @Override   
    public boolean isFinished() {
        boolean isFinished = followPathHolonomic.isFinished();
        Logger.recordOutput("DriveToPose/isFinished", isFinished);
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("DriveToPose/interrupted", interrupted);
        followPathHolonomic.cancel();
    }
}
