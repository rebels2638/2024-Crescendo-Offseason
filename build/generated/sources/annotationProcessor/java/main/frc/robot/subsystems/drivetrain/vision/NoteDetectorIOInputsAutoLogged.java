package frc.robot.subsystems.drivetrain.vision;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class NoteDetectorIOInputsAutoLogged extends NoteDetectorIO.NoteDetectorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("TxRadians", txRadians);
    table.put("TyRadians", tyRadians);
    table.put("HasTargets", hasTargets);
    table.put("BestNote", bestNote);
    table.put("TotalLatencySeconds", totalLatencySeconds);
  }

  @Override
  public void fromLog(LogTable table) {
    txRadians = table.get("TxRadians", txRadians);
    tyRadians = table.get("TyRadians", tyRadians);
    hasTargets = table.get("HasTargets", hasTargets);
    bestNote = table.get("BestNote", bestNote);
    totalLatencySeconds = table.get("TotalLatencySeconds", totalLatencySeconds);
  }

  public NoteDetectorIOInputsAutoLogged clone() {
    NoteDetectorIOInputsAutoLogged copy = new NoteDetectorIOInputsAutoLogged();
    copy.txRadians = this.txRadians;
    copy.tyRadians = this.tyRadians;
    copy.hasTargets = this.hasTargets;
    copy.bestNote = this.bestNote;
    copy.totalLatencySeconds = this.totalLatencySeconds;
    return copy;
  }
}
