package frc.robot.subsystems.drivetrain.swerve.vision;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.drivetrain.vision.NoteDetectorIO;

public class NoteDetectorIOInputsAutoLogged extends NoteDetectorIO.NoteDetectorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("VxRadians", vxRadians);
    table.put("VyRadians", vyRadians);
    table.put("HasTargets", hasTargets);
  }

  @Override
  public void fromLog(LogTable table) {
    vxRadians = table.get("VxRadians", vxRadians);
    vyRadians = table.get("VyRadians", vyRadians);
    hasTargets = table.get("HasTargets", hasTargets);
  }

  public NoteDetectorIOInputsAutoLogged clone() {
    NoteDetectorIOInputsAutoLogged copy = new NoteDetectorIOInputsAutoLogged();
    copy.vxRadians = this.vxRadians;
    copy.vyRadians = this.vyRadians;
    copy.hasTargets = this.hasTargets;
    return copy;
  }
}
