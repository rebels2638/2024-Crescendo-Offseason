package frc.robot.commands.autoAligment;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command; 

import frc.robot.Constants; 

public final class DriveToPose {

    // Builds a follow path holonomic path using field constants.
    public static Command getCommand(Pose2d endGoal) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            endGoal = GeometryUtil.flipFieldPose(endGoal);
        }

        return AutoBuilder.pathfindToPose(endGoal, new PathConstraints(Constants.Auton.MAX_MODULE_SPEED, Constants.Auton.MAX_ACCELERATION, 
            Constants.Auton.MAX_ANGULAR_VELO_RPS * Math.PI * 2, Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED * Math.PI * 2), 0, 0);
    }

}
