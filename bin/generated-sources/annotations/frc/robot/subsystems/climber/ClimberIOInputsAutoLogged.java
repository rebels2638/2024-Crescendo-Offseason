package frc.robot.subsystems.climber;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIOInputsAutoLogged extends ClimberIO.ClimberIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ClimberHeightMeters", climberHeightMeters);
    table.put("VoltageOut", voltageOut);
    table.put("ReachedSetpoint", reachedSetpoint);
    table.put("GoalPositionMeters", goalPositionMeters);
  }

  @Override
  public void fromLog(LogTable table) {
    climberHeightMeters = table.get("ClimberHeightMeters", climberHeightMeters);
    voltageOut = table.get("VoltageOut", voltageOut);
    reachedSetpoint = table.get("ReachedSetpoint", reachedSetpoint);
    goalPositionMeters = table.get("GoalPositionMeters", goalPositionMeters);
  }

  public ClimberIOInputsAutoLogged clone() {
    ClimberIOInputsAutoLogged copy = new ClimberIOInputsAutoLogged();
    copy.climberHeightMeters = this.climberHeightMeters;
    copy.voltageOut = this.voltageOut;
    copy.reachedSetpoint = this.reachedSetpoint;
    copy.goalPositionMeters = this.goalPositionMeters;
    return copy;
  }
}
