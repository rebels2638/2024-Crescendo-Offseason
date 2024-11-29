package frc.robot.subsystems.noteVisulaizer;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteVisualizer extends SubsystemBase{
    public ArrayList<Note> notes = new ArrayList<Note>();

    @Override
    public void periodic() {
        for (int i = 0; i < notes.size(); i++) {
            Translation3d noteTrans = notes.get(i).getNoteTranslation(Timer.getFPGATimestamp());
            if (noteTrans.getZ() < 0) {
                notes.remove(i);
                i--;
                continue;
            }
            Logger.recordOutput("NoteLogger/note" + i, noteTrans);
        }
    }
    
    public void addNote(Note note) {
        notes.add(note);
    }

    public static class Note {
        private final double initialvxMps;
        private final double initialvyMps;
        private final Translation3d translation;
        private final double initialLaunchTime;
        private final double robotAngle;
        private final double launchAngle; 
        private final double launchVeloMps;
        public Note(double initialLaunchTime, 
                    Translation3d translation, 
                    double robotAngle, 
                    double launchAngle,
                    double launchVeloMps,
                    double initialvxMps,
                    double initialvyMps
                    ) {
            this.translation = translation;
            this.initialLaunchTime = initialLaunchTime;
            this.robotAngle = robotAngle;
            this.launchAngle = launchAngle;
            this.launchVeloMps = launchVeloMps;
            this.initialvxMps = initialvxMps;
            this.initialvyMps = initialvyMps;
        }
        
        public Translation3d getNoteTranslation(double time) {
            double dt = time - initialLaunchTime;
            double xMeters = launchVeloMps * Math.cos(launchAngle) * dt;
            double vzMeters = launchVeloMps * Math.sin(launchAngle) * dt - .5 * 9.8 * dt * dt;
            Translation2d vxTrans = new Translation2d(xMeters, 0);
            vxTrans = vxTrans.rotateBy(new Rotation2d(robotAngle)).plus(translation.toTranslation2d());
            return new Translation3d(vxTrans.getX() + initialvxMps * dt, vxTrans.getY() + initialvyMps * dt, vzMeters + translation.getZ());
        }
        
    }
}
