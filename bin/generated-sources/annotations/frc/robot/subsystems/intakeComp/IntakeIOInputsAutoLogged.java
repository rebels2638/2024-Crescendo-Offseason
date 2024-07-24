package frc.robot.subsystems.intakeComp;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("VelocityRadSec", velocityRadSec);
    table.put("VelocityMps", velocityMps);
    table.put("DistanceMeters", distanceMeters);
    table.put("InIntake", inIntake);
    table.put("ReachedSetpoint", reachedSetpoint);
  }

  @Override
  public void fromLog(LogTable table) {
    velocityRadSec = table.get("VelocityRadSec", velocityRadSec);
    velocityMps = table.get("VelocityMps", velocityMps);
    distanceMeters = table.get("DistanceMeters", distanceMeters);
    inIntake = table.get("InIntake", inIntake);
    reachedSetpoint = table.get("ReachedSetpoint", reachedSetpoint);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.velocityRadSec = this.velocityRadSec;
    copy.velocityMps = this.velocityMps;
    copy.distanceMeters = this.distanceMeters;
    copy.inIntake = this.inIntake;
    copy.reachedSetpoint = this.reachedSetpoint;
    return copy;
  }
}
