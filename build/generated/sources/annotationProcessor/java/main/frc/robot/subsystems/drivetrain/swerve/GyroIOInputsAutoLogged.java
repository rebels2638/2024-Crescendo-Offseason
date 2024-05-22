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
  }

  @Override
  public void fromLog(LogTable table) {
    yaw = table.get("Yaw", yaw);
    isConnected = table.get("IsConnected", isConnected);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.yaw = this.yaw;
    copy.isConnected = this.isConnected;
    return copy;
  }
}
