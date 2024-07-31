package frc.robot.commands.autoAligment;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
 

public class DriveToPose extends Command {
    private Command followPathHolonomic;

    private final Pose2d endGoal;

    public DriveToPose(Pose2d endGoal) {
        this.endGoal = endGoal;
    }

    // Builds a follow path holonomic path using field constants.
    @Override
    public void initialize() {
        followPathHolonomic = AutoBuilder.pathfindToPose(endGoal, new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION, 
            Constants.Auton.MAX_ANGULAR_VELO_RPS, Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED), 0, 0);
        CommandScheduler.getInstance().schedule(followPathHolonomic);
    }

    @Override   
    public boolean isFinished() {
        System.out.println(followPathHolonomic.isFinished());
        return followPathHolonomic.isFinished();
    }
}
