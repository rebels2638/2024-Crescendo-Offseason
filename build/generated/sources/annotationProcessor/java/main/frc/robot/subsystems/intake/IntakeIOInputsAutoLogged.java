package frc.robot.subsystems.intake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IntakeVelocityMps", intakeVelocityMps);
    table.put("IntakePoseMeters", intakePoseMeters);
    table.put("Temp", temp);
    table.put("Volts", volts);
    table.put("Amps", amps);
    table.put("InIntake", inIntake);
  }

  @Override
  public void fromLog(LogTable table) {
    intakeVelocityMps = table.get("IntakeVelocityMps", intakeVelocityMps);
    intakePoseMeters = table.get("IntakePoseMeters", intakePoseMeters);
    temp = table.get("Temp", temp);
    volts = table.get("Volts", volts);
    amps = table.get("Amps", amps);
    inIntake = table.get("InIntake", inIntake);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.intakeVelocityMps = this.intakeVelocityMps;
    copy.intakePoseMeters = this.intakePoseMeters;
    copy.temp = this.temp;
    copy.volts = this.volts;
    copy.amps = this.amps;
    copy.inIntake = this.inIntake;
    return copy;
  }
}
