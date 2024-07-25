package frc.robot.subsystems.elevator;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ShooterHeightMeters", shooterHeightMeters);
    table.put("ClimberHeightMeters", climberHeightMeters);
    table.put("VoltageOut", voltageOut);
    table.put("ReachedSetpoint", reachedSetpoint);
    table.put("GoalPositionMeters", goalPositionMeters);
  }

  @Override
  public void fromLog(LogTable table) {
    shooterHeightMeters = table.get("ShooterHeightMeters", shooterHeightMeters);
    climberHeightMeters = table.get("ClimberHeightMeters", climberHeightMeters);
    voltageOut = table.get("VoltageOut", voltageOut);
    reachedSetpoint = table.get("ReachedSetpoint", reachedSetpoint);
    goalPositionMeters = table.get("GoalPositionMeters", goalPositionMeters);
  }

  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.shooterHeightMeters = this.shooterHeightMeters;
    copy.climberHeightMeters = this.climberHeightMeters;
    copy.voltageOut = this.voltageOut;
    copy.reachedSetpoint = this.reachedSetpoint;
    copy.goalPositionMeters = this.goalPositionMeters;
    return copy;
  }
}
