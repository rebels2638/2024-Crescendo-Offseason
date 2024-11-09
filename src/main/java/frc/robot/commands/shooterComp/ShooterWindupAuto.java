package frc.robot.commands.shooterComp;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooterComp.Shooter;

public class ShooterWindupAuto extends Command {
    
    private double velocitySetPoint = 60;
    private final Shooter shooterSubsystem = Shooter.getInstance();

    public ShooterWindupAuto(double setpoint){
        this.velocitySetPoint = setpoint;
    }

    public ShooterWindupAuto() {
    }
    public ShooterWindupAuto(SequentialCommandGroup s){
        s.cancel();
    }

    @Override
    public void initialize(){
        shooterSubsystem.setVelocityRadSec(velocitySetPoint, true, velocitySetPoint - 10, velocitySetPoint);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
