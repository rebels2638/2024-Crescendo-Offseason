package frc.robot.subsystems.indexer;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IndexerIOInputsAutoLogged extends IndexerIO.IndexerIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("InIntake", inIntake);
    table.put("InShooter", inShooter);
  }

  @Override
  public void fromLog(LogTable table) {
    inIntake = table.get("InIntake", inIntake);
    inShooter = table.get("InShooter", inShooter);
  }

  public IndexerIOInputsAutoLogged clone() {
    IndexerIOInputsAutoLogged copy = new IndexerIOInputsAutoLogged();
    copy.inIntake = this.inIntake;
    copy.inShooter = this.inShooter;
    return copy;
  }
}
