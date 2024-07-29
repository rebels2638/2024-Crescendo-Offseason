package frc.robot.subsystems.drivetrain.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NoteDetectorIOReal implements NoteDetectorIO {
    private NetworkTable llTable;
    public NoteDetectorIOReal() {
        llTable = NetworkTableInstance.getDefault().getTable("limelight");

    }
    public void updateInputs(NoteDetectorIOInputs inputs) {
        inputs.hasTargets = llTable.getEntry("tv").getDouble(0) == 1;
        inputs.txRadians = -Math.toRadians(llTable.getEntry("tx").getDouble(0));
        inputs.tyRadians = Math.toRadians(llTable.getEntry("ty").getDouble(0));
    }

}