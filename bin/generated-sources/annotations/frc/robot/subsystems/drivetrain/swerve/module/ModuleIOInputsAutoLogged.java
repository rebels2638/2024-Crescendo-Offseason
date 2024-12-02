package frc.robot.subsystems.drivetrain.swerve.module;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Timestamp", timestamp);
    table.put("DrivePositionRad", drivePositionRad);
    table.put("DriveVelocityRadPerSec", driveVelocityRadPerSec);
    table.put("DriveCurrentAmps", driveCurrentAmps);
    table.put("DriveAppliedVolts", driveAppliedVolts);
    table.put("DriveTemperature", driveTemperature);
    table.put("SteerAbsolutePosition", steerAbsolutePosition);
    table.put("SteerAbsolutePositionRadians", steerAbsolutePositionRadians);
    table.put("SteerPosition", steerPosition);
    table.put("SteerPositionRad", steerPositionRad);
    table.put("SteerVelocityRadPerSec", steerVelocityRadPerSec);
    table.put("SteerCurrentDrawAmps", steerCurrentDrawAmps);
    table.put("SteerAppliedVolts", steerAppliedVolts);
    table.put("SteerTemperature", steerTemperature);
    table.put("DesiredSteerPositionRad", desiredSteerPositionRad);
    table.put("DesiredDriveVelocityMetersPerSec", desiredDriveVelocityMetersPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    timestamp = table.get("Timestamp", timestamp);
    drivePositionRad = table.get("DrivePositionRad", drivePositionRad);
    driveVelocityRadPerSec = table.get("DriveVelocityRadPerSec", driveVelocityRadPerSec);
    driveCurrentAmps = table.get("DriveCurrentAmps", driveCurrentAmps);
    driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
    driveTemperature = table.get("DriveTemperature", driveTemperature);
    steerAbsolutePosition = table.get("SteerAbsolutePosition", steerAbsolutePosition);
    steerAbsolutePositionRadians = table.get("SteerAbsolutePositionRadians", steerAbsolutePositionRadians);
    steerPosition = table.get("SteerPosition", steerPosition);
    steerPositionRad = table.get("SteerPositionRad", steerPositionRad);
    steerVelocityRadPerSec = table.get("SteerVelocityRadPerSec", steerVelocityRadPerSec);
    steerCurrentDrawAmps = table.get("SteerCurrentDrawAmps", steerCurrentDrawAmps);
    steerAppliedVolts = table.get("SteerAppliedVolts", steerAppliedVolts);
    steerTemperature = table.get("SteerTemperature", steerTemperature);
    desiredSteerPositionRad = table.get("DesiredSteerPositionRad", desiredSteerPositionRad);
    desiredDriveVelocityMetersPerSec = table.get("DesiredDriveVelocityMetersPerSec", desiredDriveVelocityMetersPerSec);
  }

  public ModuleIOInputsAutoLogged clone() {
    ModuleIOInputsAutoLogged copy = new ModuleIOInputsAutoLogged();
    copy.timestamp = this.timestamp;
    copy.drivePositionRad = this.drivePositionRad;
    copy.driveVelocityRadPerSec = this.driveVelocityRadPerSec;
    copy.driveCurrentAmps = this.driveCurrentAmps;
    copy.driveAppliedVolts = this.driveAppliedVolts;
    copy.driveTemperature = this.driveTemperature;
    copy.steerAbsolutePosition = this.steerAbsolutePosition;
    copy.steerAbsolutePositionRadians = this.steerAbsolutePositionRadians;
    copy.steerPosition = this.steerPosition;
    copy.steerPositionRad = this.steerPositionRad;
    copy.steerVelocityRadPerSec = this.steerVelocityRadPerSec;
    copy.steerCurrentDrawAmps = this.steerCurrentDrawAmps;
    copy.steerAppliedVolts = this.steerAppliedVolts;
    copy.steerTemperature = this.steerTemperature;
    copy.desiredSteerPositionRad = this.desiredSteerPositionRad;
    copy.desiredDriveVelocityMetersPerSec = this.desiredDriveVelocityMetersPerSec;
    return copy;
  }
}
