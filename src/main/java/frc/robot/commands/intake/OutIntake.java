package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeComp.Intake;

public class OutIntake extends Command{
    private Intake intakeNeo = Intake.getInstance();
    private boolean isIn = false;

    public OutIntake() {
    }
    
    @Override
    public void execute(){
        // System.out.println("Out intake : " + !isIn);
        isIn = intakeNeo.inIntake();
    }
    
    @Override
    public void end(boolean isInterrupted){
        return;
    }
    
    @Override
    public boolean isFinished(){
        return !isIn;
    }
}
