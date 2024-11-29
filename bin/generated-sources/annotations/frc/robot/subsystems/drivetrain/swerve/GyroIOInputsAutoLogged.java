package frc.robot.subsystems.drivetrain.swerve;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Yaw", yaw);
    table.put("IsConnected", isConnected);
    table.put("YawRadSec", yawRadSec);
  }

  @Override
  public void fromLog(LogTable table) {
    yaw = table.get("Yaw", yaw);
    isConnected = table.get("IsConnected", isConnected);
    yawRadSec = table.get("YawRadSec", yawRadSec);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.yaw = this.yaw;
    copy.isConnected = this.isConnected;
    copy.yawRadSec = this.yawRadSec;
    return copy;
  }
}
