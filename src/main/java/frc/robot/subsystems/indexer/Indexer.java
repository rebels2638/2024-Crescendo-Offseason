package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.intake.Intake;

public class Indexer extends SubsystemBase {
    private IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final SwerveDrive swerveDrive;
    private Intake intake;
    public Indexer(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }
    
    public boolean inIntake() {
        return inputs.inIntake;
    }

    public boolean inShooter() {
        return inputs.inShooter;
    }

    public void setIntake(Intake intake) {
        this.intake = intake;
        switch (Constants.currentMode) {
            case SIM:
                io = new IndexerIOSim(swerveDrive, intake);
                break;
        
            default:
                io = new IndexerIOReal();
                break;
        }
    }
}