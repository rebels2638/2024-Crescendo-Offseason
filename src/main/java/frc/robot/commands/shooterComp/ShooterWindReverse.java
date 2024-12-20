package frc.robot.commands.shooterComp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterComp.Shooter;

public class ShooterWindReverse extends Command {
    private double velocitySetPoint = -24;
    private final Shooter shooterSubsystem = Shooter.getInstance();

    public ShooterWindReverse(){
    }

    @Override
    public void initialize(){
        shooterSubsystem.setVelocityRadSec(velocitySetPoint, false, 0, 0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
