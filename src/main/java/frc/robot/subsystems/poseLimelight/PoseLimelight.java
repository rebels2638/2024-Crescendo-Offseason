package frc.robot.subsystems.poseLimelight;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseLimelight extends SubsystemBase {
    private final PoseLimelightIOInputsAutoLogged inputs = new PoseLimelightIOInputsAutoLogged();
    private final PoseLimelightIO io;
    
    private Queue<Double[]>[] tagQueue = new LinkedList[14];
    private double prevTime = 0;

    public PoseLimelight() {
        switch (Constants.currentMode) {
            case REAL:
                io = new PoseLimelightIOReal();
                break;

            case SIM:
                io = new PoseLimelightIOSim();
                break;

            default:
                io = new PoseLimelightIO() {};
                break;
        }

        for (int i = 0; i < 14; i++) {
            tagQueue[i] = new LinkedList<Double[]>();
        }
        
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("PoseLimelight", inputs);

        for (int i = 0; i < 14; i++) {
            // tx, ty, isSeen (1 or 0), timestamp seconds
            tagQueue[i].add(new Double[] {
                Double.valueOf(inputs.tx), 
                Double.valueOf(inputs.ty), 
                Double.valueOf(inputs.hasValidTargets? 1 : 0), 
                Double.valueOf(inputs.timestampSeconds)});

            if (Timer.getFPGATimestamp() - tagQueue[i].peek()[3].doubleValue() > 1) {
                tagQueue[i].poll();
            }
        }

        prevTime = Timer.getFPGATimestamp();
        // Logger.recordOutput("PoseLimelight/estRobotPose", getEstimatedRobotPose());
    }

    // public double getPrimaryIDMovment() {
    //     Queue<Double[]> queue = tagQueue[inputs.primaryTagId];
    //     List<Double[]> list = new ArrayList<Double[]>(queue);
    //     double lastTime = 0; 
        
    //     for (int i = list.size() - 1; i >= 0; i--) {
    //         double dt = list.get(list.size() - 1)[3].doubleValue() - list.get(i)[3].doubleValue();
    //         if (list.get(i)[2].doubleValue() == 1 && dt <= 0.2) {
    //             return Math.hypot(, i) / list.get(i)
    //         }
    //     }
    // }

    public boolean hasValidTargets() {
        return inputs.hasValidTargets;
    }
    
    public Pose2d getEstimatedRobotPose() {    
        return inputs.estimatedPose;
    }
    
    public double getTimestampSeconds() {
        return inputs.timestampSeconds;
    }
}