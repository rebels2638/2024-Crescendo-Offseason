package frc.robot.subsystems.drivetrain.vision;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NoteDetectorIOReal implements NoteDetectorIO {
    private NetworkTable llTable;
    private static final Debouncer debouncer = new Debouncer(0.2);

    public NoteDetectorIOReal() {
        llTable = NetworkTableInstance.getDefault().getTable("limelight");

    }
    public void updateInputs(NoteDetectorIOInputs inputs) {
        inputs.hasTargets = debouncer.calculate(llTable.getEntry("tv").getDouble(0) == 1);
        inputs.txRadians = -Math.toRadians(llTable.getEntry("tx").getDouble(0));
        inputs.tyRadians = Math.toRadians(llTable.getEntry("ty").getDouble(0));
        inputs.totalLatencySeconds = (llTable.getEntry("cl").getDouble(0) + llTable.getEntry("tl").getDouble(0))/1000;
    }

}