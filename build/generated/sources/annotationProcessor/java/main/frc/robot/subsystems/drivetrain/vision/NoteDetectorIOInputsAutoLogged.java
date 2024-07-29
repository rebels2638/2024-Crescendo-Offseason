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
  }

  @Override
  public void fromLog(LogTable table) {
    txRadians = table.get("TxRadians", txRadians);
    tyRadians = table.get("TyRadians", tyRadians);
    hasTargets = table.get("HasTargets", hasTargets);
  }

  public NoteDetectorIOInputsAutoLogged clone() {
    NoteDetectorIOInputsAutoLogged copy = new NoteDetectorIOInputsAutoLogged();
    copy.txRadians = this.txRadians;
    copy.tyRadians = this.tyRadians;
    copy.hasTargets = this.hasTargets;
    return copy;
  }
}
