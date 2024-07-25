package frc.robot.subsystems.shooterComp;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterIOInputsAutoLogged extends ShooterIO.ShooterIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("VelocityRadSec", velocityRadSec);
    table.put("DesiredVelocityRadSec", desiredVelocityRadSec);
    table.put("ReachedSetpoint", reachedSetpoint);
    table.put("InShooter", inShooter);
  }

  @Override
  public void fromLog(LogTable table) {
    velocityRadSec = table.get("VelocityRadSec", velocityRadSec);
    desiredVelocityRadSec = table.get("DesiredVelocityRadSec", desiredVelocityRadSec);
    reachedSetpoint = table.get("ReachedSetpoint", reachedSetpoint);
    inShooter = table.get("InShooter", inShooter);
  }

  public ShooterIOInputsAutoLogged clone() {
    ShooterIOInputsAutoLogged copy = new ShooterIOInputsAutoLogged();
    copy.velocityRadSec = this.velocityRadSec;
    copy.desiredVelocityRadSec = this.desiredVelocityRadSec;
    copy.reachedSetpoint = this.reachedSetpoint;
    copy.inShooter = this.inShooter;
    return copy;
  }
}
