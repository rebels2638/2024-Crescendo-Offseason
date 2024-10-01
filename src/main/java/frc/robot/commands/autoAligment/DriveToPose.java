package frc.robot.commands.autoAligment;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import com.pathplanner.lib.util.GeometryUtil;
 

public final class DriveToPose {

    // Builds a follow path holonomic path using field constants.
    public static Command getCommand(Pose2d endGoal) {
        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        //     endGoal = GeometryUtil.flipFieldPose(endGoal);
        // } 
        
        return AutoBuilder.pathfindToPose(endGoal, new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION, 
            Constants.Auton.MAX_ANGULAR_VELO_RPS, Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED), 0, 0);
    }
}
