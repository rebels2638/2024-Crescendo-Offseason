package frc.robot.subsystems.poseLimelight;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PoseLimelightIOInputsAutoLogged extends PoseLimelightIO.PoseLimelightIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("EstimatedPose", estimatedPose);
    table.put("TimestampSeconds", timestampSeconds);
    table.put("HasValidTargets", hasValidTargets);
    table.put("PrimaryTagId", primaryTagId);
    table.put("Tx", tx);
    table.put("Ty", ty);
  }

  @Override
  public void fromLog(LogTable table) {
    estimatedPose = table.get("EstimatedPose", estimatedPose);
    timestampSeconds = table.get("TimestampSeconds", timestampSeconds);
    hasValidTargets = table.get("HasValidTargets", hasValidTargets);
    primaryTagId = table.get("PrimaryTagId", primaryTagId);
    tx = table.get("Tx", tx);
    ty = table.get("Ty", ty);
  }

  public PoseLimelightIOInputsAutoLogged clone() {
    PoseLimelightIOInputsAutoLogged copy = new PoseLimelightIOInputsAutoLogged();
    copy.estimatedPose = this.estimatedPose;
    copy.timestampSeconds = this.timestampSeconds;
    copy.hasValidTargets = this.hasValidTargets;
    copy.primaryTagId = this.primaryTagId;
    copy.tx = this.tx;
    copy.ty = this.ty;
    return copy;
  }
}
