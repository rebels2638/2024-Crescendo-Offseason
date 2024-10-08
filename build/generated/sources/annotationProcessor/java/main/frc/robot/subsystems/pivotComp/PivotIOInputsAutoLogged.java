package frc.robot.subsystems.pivotComp;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PivotIOInputsAutoLogged extends PivotIO.PivotIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PositionRad", positionRad);
    table.put("PositionDeg", positionDeg);
    table.put("VelocityDegSec", velocityDegSec);
    table.put("VelocityRadSec", velocityRadSec);
    table.put("ReachedSetpoint", reachedSetpoint);
  }

  @Override
  public void fromLog(LogTable table) {
    positionRad = table.get("PositionRad", positionRad);
    positionDeg = table.get("PositionDeg", positionDeg);
    velocityDegSec = table.get("VelocityDegSec", velocityDegSec);
    velocityRadSec = table.get("VelocityRadSec", velocityRadSec);
    reachedSetpoint = table.get("ReachedSetpoint", reachedSetpoint);
  }

  public PivotIOInputsAutoLogged clone() {
    PivotIOInputsAutoLogged copy = new PivotIOInputsAutoLogged();
    copy.positionRad = this.positionRad;
    copy.positionDeg = this.positionDeg;
    copy.velocityDegSec = this.velocityDegSec;
    copy.velocityRadSec = this.velocityRadSec;
    copy.reachedSetpoint = this.reachedSetpoint;
    return copy;
  }
}
