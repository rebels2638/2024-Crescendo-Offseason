package frc.robot.subsystems.drivetrain.swerve;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DriveVelocityMps", driveVelocityMps);
    table.put("DrivePositionMeters", drivePositionMeters);
    table.put("AngleVelocityRadPerSec", angleVelocityRadPerSec);
    table.put("AnglePositionRad", anglePositionRad);
    table.put("DriveCurrentAmps", driveCurrentAmps);
    table.put("AngleCurrentAmps", angleCurrentAmps);
    table.put("DriveTempC", driveTempC);
    table.put("AngleTempC", angleTempC);
    table.put("DriveVoltage", driveVoltage);
    table.put("AngleVoltage", angleVoltage);
  }

  @Override
  public void fromLog(LogTable table) {
    driveVelocityMps = table.get("DriveVelocityMps", driveVelocityMps);
    drivePositionMeters = table.get("DrivePositionMeters", drivePositionMeters);
    angleVelocityRadPerSec = table.get("AngleVelocityRadPerSec", angleVelocityRadPerSec);
    anglePositionRad = table.get("AnglePositionRad", anglePositionRad);
    driveCurrentAmps = table.get("DriveCurrentAmps", driveCurrentAmps);
    angleCurrentAmps = table.get("AngleCurrentAmps", angleCurrentAmps);
    driveTempC = table.get("DriveTempC", driveTempC);
    angleTempC = table.get("AngleTempC", angleTempC);
    driveVoltage = table.get("DriveVoltage", driveVoltage);
    angleVoltage = table.get("AngleVoltage", angleVoltage);
  }

  public ModuleIOInputsAutoLogged clone() {
    ModuleIOInputsAutoLogged copy = new ModuleIOInputsAutoLogged();
    copy.driveVelocityMps = this.driveVelocityMps;
    copy.drivePositionMeters = this.drivePositionMeters;
    copy.angleVelocityRadPerSec = this.angleVelocityRadPerSec;
    copy.anglePositionRad = this.anglePositionRad;
    copy.driveCurrentAmps = this.driveCurrentAmps;
    copy.angleCurrentAmps = this.angleCurrentAmps;
    copy.driveTempC = this.driveTempC;
    copy.angleTempC = this.angleTempC;
    copy.driveVoltage = this.driveVoltage;
    copy.angleVoltage = this.angleVoltage;
    return copy;
  }
}
