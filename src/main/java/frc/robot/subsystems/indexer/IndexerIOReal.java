package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOReal implements IndexerIO {

    private final DigitalInput intakeBrake = new DigitalInput(7);
    private final DigitalInput shooterBrake = new DigitalInput(9);
    public IndexerIOReal() {
        
    }
    
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.inIntake = !intakeBrake.get();
        inputs.inShooter = !shooterBrake.get();
    }
}
