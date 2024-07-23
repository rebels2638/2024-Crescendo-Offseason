package frc.robot.subsystems.drivetrain.indexer;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.indexer.IndexerIO;

public class IndexerIOInputsAutoLogged extends IndexerIO.IndexerIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("InIntake", inIntake);
  }

  @Override
  public void fromLog(LogTable table) {
    inIntake = table.get("InIntake", inIntake);
  }

  public IndexerIOInputsAutoLogged clone() {
    IndexerIOInputsAutoLogged copy = new IndexerIOInputsAutoLogged();
    copy.inIntake = this.inIntake;
    return copy;
  }
}
