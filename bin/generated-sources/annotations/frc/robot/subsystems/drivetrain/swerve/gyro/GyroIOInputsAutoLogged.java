package frc.robot.subsystems.drivetrain.swerve.gyro;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("Yaw", yaw);
    table.put("Pitch", pitch);
    table.put("Roll", roll);
    table.put("AngularVelocity", angularVelocity);
    table.put("AngularVelocityDegrees", angularVelocityDegrees);
    table.put("AccelerationX", accelerationX);
    table.put("AccelerationY", accelerationY);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    yaw = table.get("Yaw", yaw);
    pitch = table.get("Pitch", pitch);
    roll = table.get("Roll", roll);
    angularVelocity = table.get("AngularVelocity", angularVelocity);
    angularVelocityDegrees = table.get("AngularVelocityDegrees", angularVelocityDegrees);
    accelerationX = table.get("AccelerationX", accelerationX);
    accelerationY = table.get("AccelerationY", accelerationY);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.yaw = this.yaw;
    copy.pitch = this.pitch;
    copy.roll = this.roll;
    copy.angularVelocity = this.angularVelocity;
    copy.angularVelocityDegrees = this.angularVelocityDegrees;
    copy.accelerationX = this.accelerationX;
    copy.accelerationY = this.accelerationY;
    return copy;
  }
}
