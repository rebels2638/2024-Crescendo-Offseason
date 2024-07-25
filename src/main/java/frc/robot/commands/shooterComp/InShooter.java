package frc.robot.commands.shooterComp;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterComp.Shooter;

public class InShooter extends Command{
    private boolean isIn = false;

    public InShooter(){
    }
    
    @Override
    public void execute(){
        isIn = Shooter.getInstance().inShooter();
    }
    
    @Override
    public void end(boolean isInterrupted){
        return;
    }
    
    @Override
    public boolean isFinished(){
        return isIn;
    }
}
