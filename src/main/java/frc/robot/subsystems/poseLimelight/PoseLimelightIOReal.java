package frc.robot.subsystems.poseLimelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.lib.util.LimelightHelpers;

public class PoseLimelightIOReal implements PoseLimelightIO {
    public PoseLimelightIOReal() {

    }
    public void updateInputs(PoseLimelightIOInputs inputs) {
        // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-tag");
        // inputs.estimatedPose = mt2.pose;
        // inputs.timestampSeconds = mt2.timestampSeconds;
        // inputs.hasValidTargets = mt2.tagCount > 0;
        // inputs.primaryTagId = (int) NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("tid").getDouble(0);
        // inputs.tx = NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("tx").getDouble(0);
        // inputs.ty = NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("ty").getDouble(0);


    }

}