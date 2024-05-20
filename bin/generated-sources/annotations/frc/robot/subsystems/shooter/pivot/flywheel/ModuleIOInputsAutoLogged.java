package frc.robot.subsystems.shooter.pivot.flywheel;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInputsAutoLogged extends FlywheeIO.ModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Rpm", rpm);
    table.put("Voltage", voltage);
    table.put("Amps", amps);
    table.put("Temp", temp);
  }

  @Override
  public void fromLog(LogTable table) {
    rpm = table.get("Rpm", rpm);
    voltage = table.get("Voltage", voltage);
    amps = table.get("Amps", amps);
    temp = table.get("Temp", temp);
  }

  public ModuleIOInputsAutoLogged clone() {
    ModuleIOInputsAutoLogged copy = new ModuleIOInputsAutoLogged();
    copy.rpm = this.rpm;
    copy.voltage = this.voltage;
    copy.amps = this.amps;
    copy.temp = this.temp;
    return copy;
  }
}
