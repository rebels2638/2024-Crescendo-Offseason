package frc.robot.commands.autoAligment;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints; 

import edu.wpi.first.math.geometry.Pose2d; 

import edu.wpi.first.wpilibj2.command.Command; 

import frc.robot.Constants; 

public final class DriveToPose {

    // Builds a follow path holonomic path using field constants.
    public static Command getCommand(Pose2d endGoal) {

        // Uncomment the following block to flip the end goal based on the robot's alliance color.
        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        //     endGoal = GeometryUtil.flipFieldPose(endGoal); // Flip the pose if the robot is on the red alliance.
        // } 

        // Create and return a command to pathfind to the specified end goal pose.
        return AutoBuilder.pathfindToPose(endGoal, 
            new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION, 
                Constants.Auton.MAX_ANGULAR_VELO_RPS, Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED), 
            0, 0); // Using specified max speed, acceleration, and angular constraints for pathfinding.
    }

}
