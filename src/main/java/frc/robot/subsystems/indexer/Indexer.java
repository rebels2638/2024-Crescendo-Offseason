package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.drivetrain.vision.NoteDetector;
import frc.robot.subsystems.intakeComp.Intake;
import frc.robot.subsystems.pivotComp.Pivot;

public class Indexer extends SubsystemBase {
    private IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final SwerveDrive swerveDrive;
    private Intake intake;
    private final NoteDetector noteDetector;
    public Indexer(SwerveDrive swerveDrive, NoteDetector noteDetector) {
        this.swerveDrive = swerveDrive;
        this.noteDetector = noteDetector;
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

    public void setSubsystem(Intake intake, Pivot pivot) {
        this.intake = intake;
        switch (Constants.currentMode) {
            case SIM:
                io = new IndexerIOSim(swerveDrive, intake, noteDetector, pivot);
                break;
        
            default:
                io = new IndexerIOReal();
                break;
        }
    }
}
