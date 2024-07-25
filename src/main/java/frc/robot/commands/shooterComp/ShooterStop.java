package frc.robot.commands.shooterComp;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooterComp.Shooter;

public class ShooterStop extends Command {
    // private double velocitySetPoint = 0;
    private final Shooter shooterSubsystem = Shooter.getInstance();

    public ShooterStop(){
    }
    public ShooterStop(SequentialCommandGroup s){
        s.cancel();
    }

    @Override
    public void initialize(){
        shooterSubsystem.setVelocityRadSec(0,false, 0, 0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
