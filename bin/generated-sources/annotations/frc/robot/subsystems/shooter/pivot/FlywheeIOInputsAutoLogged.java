package frc.robot.subsystems.shooter.pivot;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FlywheeIOInputsAutoLogged extends PivotIO.FlywheeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("AngleRad", angleRad);
    table.put("TTemp", tTemp);
    table.put("BTemp", bTemp);
    table.put("TAmps", tAmps);
    table.put("BAmps", bAmps);
    table.put("TVolts", tVolts);
    table.put("BVolts", bVolts);
  }

  @Override
  public void fromLog(LogTable table) {
    angleRad = table.get("AngleRad", angleRad);
    tTemp = table.get("TTemp", tTemp);
    bTemp = table.get("BTemp", bTemp);
    tAmps = table.get("TAmps", tAmps);
    bAmps = table.get("BAmps", bAmps);
    tVolts = table.get("TVolts", tVolts);
    bVolts = table.get("BVolts", bVolts);
  }

  public FlywheeIOInputsAutoLogged clone() {
    FlywheeIOInputsAutoLogged copy = new FlywheeIOInputsAutoLogged();
    copy.angleRad = this.angleRad;
    copy.tTemp = this.tTemp;
    copy.bTemp = this.bTemp;
    copy.tAmps = this.tAmps;
    copy.bAmps = this.bAmps;
    copy.tVolts = this.tVolts;
    copy.bVolts = this.bVolts;
    return copy;
  }
}
