package frc.robot.subsystems.drivetrain.swerve;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DriveVelocityMps", driveVelocityMps);
    table.put("DrivePositionsMeters", drivePositionsMeters);
    table.put("AnglePoseRad", anglePoseRad);
  }

  @Override
  public void fromLog(LogTable table) {
    driveVelocityMps = table.get("DriveVelocityMps", driveVelocityMps);
    drivePositionsMeters = table.get("DrivePositionsMeters", drivePositionsMeters);
    anglePoseRad = table.get("AnglePoseRad", anglePoseRad);
  }

  public ModuleIOInputsAutoLogged clone() {
    ModuleIOInputsAutoLogged copy = new ModuleIOInputsAutoLogged();
    copy.driveVelocityMps = this.driveVelocityMps;
    copy.drivePositionsMeters = this.drivePositionsMeters;
    copy.anglePoseRad = this.anglePoseRad;
    return copy;
  }
}
