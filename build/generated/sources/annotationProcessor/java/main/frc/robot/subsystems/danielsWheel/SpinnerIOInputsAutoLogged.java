package frc.robot.subsystems.danielsWheel;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SpinnerIOInputsAutoLogged extends SpinnerIO.SpinnerIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Velocity", velocity);
  }

  @Override
  public void fromLog(LogTable table) {
    velocity = table.get("Velocity", velocity);
  }

  public SpinnerIOInputsAutoLogged clone() {
    SpinnerIOInputsAutoLogged copy = new SpinnerIOInputsAutoLogged();
    copy.velocity = this.velocity;
    return copy;
  }
}
