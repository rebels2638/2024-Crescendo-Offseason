package frc.robot.commands.shooterComp;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooterComp.Shooter;

public class ShooterAmp extends Command {
    
    private double topVelo = 10;
    private double bottomVelo = 60;
    private final Shooter shooterSubsystem = Shooter.getInstance();

    @Override
    public void initialize(){
        shooterSubsystem.setVelocityRadSec(0, true, bottomVelo, topVelo);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
