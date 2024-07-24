package frc.robot.subsystems.intakeComp;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("VelocityRadSec", velocityRadSec);
    table.put("InIntake", inIntake);
    table.put("ReachedSetpoint", reachedSetpoint);
  }

  @Override
  public void fromLog(LogTable table) {
    velocityRadSec = table.get("VelocityRadSec", velocityRadSec);
    inIntake = table.get("InIntake", inIntake);
    reachedSetpoint = table.get("ReachedSetpoint", reachedSetpoint);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.velocityRadSec = this.velocityRadSec;
    copy.inIntake = this.inIntake;
    copy.reachedSetpoint = this.reachedSetpoint;
    return copy;
  }
}
