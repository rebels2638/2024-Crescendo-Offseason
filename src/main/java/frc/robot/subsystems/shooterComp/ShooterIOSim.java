package frc.robot.subsystems.shooterComp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.Indexer;

public class ShooterIOSim extends SubsystemBase implements ShooterIO {

    double desiredVelocityRadSec = 0;

    private final Indexer indexer;
    public ShooterIOSim(Indexer indexer) {
        this.indexer = indexer;
    }
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.velocityRadSec = desiredVelocityRadSec;
        inputs.inShooter = indexer.inShooter();
    }

    @Override
    // sould be called periodically
    public void setVelocityRadSec(double goalVelocityRadPerSec, boolean isVar, double Bottom, double Top) {
        desiredVelocityRadSec = goalVelocityRadPerSec;
    } 

    @Override
    public void setVoltage(double voltage){
    }

    @Override
    public void configureController(SimpleMotorFeedforward vff, PIDController vfb ) {
    }

}