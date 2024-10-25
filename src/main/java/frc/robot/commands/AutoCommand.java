package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.commands.autoAligment.DriveToPose;
import frc.robot.commands.autoAligment.NotePresent;
import frc.robot.commands.compositions.IntakeNote;
import frc.robot.commands.compositions.ShootNoteAuto;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.drivetrain.vision.NoteDetector;
import frc.robot.subsystems.intakeComp.Intake;

public class AutoCommand extends Command {
    private final String[] args;
    private final SwerveDrive swerveDrive;
    private final Intake intake;
    private final NoteDetector noteDetector;
    private boolean isRed = false;

    private Command currentCommand = new Command() {
        @Override
        public boolean isFinished() {
            return true; // i had no better way of doing this \_^_^_/ 
        }
    };

    private int i = 0;

    /* 
    ARG: Array of Strings representing tasks for the funcion to composit into a auto.

    It is important that all paths are properly named to convetion for the parser to work properly.

    Follow a path -> pass the path name as used in pathplanner.
        If this is a "To" path, the next arg needs to be the indended note and follow with the desired fallbacks ie. amp, mid, source, far1, ... far5.
        These fallbacks will be executed in the order they are provided.
    Score -> "score"

    Example: { "ToFar1FromAmp", "far1", far2", "far3", "FromFar1ToAmp", "score" }
    */
    public AutoCommand(String[] args, SwerveDrive swerveDrive, Intake intake, NoteDetector noteDetector) {
        this.args = args;
        this.swerveDrive = swerveDrive;
        this.noteDetector = noteDetector;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isRed =  alliance.get() == DriverStation.Alliance.Red;
        }

        PathPlannerPath path = PathPlannerPath.fromPathFile(args[0]);
        if (isRed) {
            path = path.flipPath();

            Pose2d p = path.getPreviewStartingHolonomicPose();
            new InstantCommand(() -> swerveDrive.resetPose(
                new Pose2d(p.getTranslation(), 
                new Rotation2d(p.getRotation().unaryMinus().getRadians() + Math.PI / 2 + Math.PI)))
            ).schedule();
        }
        else {
            Pose2d p = path.getPreviewStartingHolonomicPose();
            new InstantCommand(() -> swerveDrive.resetPose(
                new Pose2d(p.getTranslation(), 
                new Rotation2d(p.getRotation().unaryMinus().getRadians() + Math.PI / 2)))
            ).schedule();
        }

        

    }
    @Override
    public void execute() {
        Logger.recordOutput("AutoCommand/currentCommand.isFinished()", currentCommand.isFinished());
        if (i >= args.length) {
            return;
        }
        
        if (!currentCommand.isFinished()) { 
            return; 
        }

        String cmd = args[i];
        Logger.recordOutput("AutoCommand/cmd", cmd);

        if (cmd == "score") {
            Logger.recordOutput("AutoCommand/currentCommand", "score");

            currentCommand = new ShootNoteAuto();
        }


        else if (cmd.substring(0, 2).equals("To") || 
                 cmd.substring(0, 2).equals("Fr")) {
            Logger.recordOutput("AutoCommand/currentCommand", "driveToNote");

            PathPlannerPath path = PathPlannerPath.fromPathFile(cmd);
            currentCommand = AutoBuilder.followPath(path);
        }
        
        else if (NotePresent.notePresent(noteDetector, intake, swerveDrive, getIndex(cmd), false)) {
            Logger.recordOutput("AutoCommand/currentCommand", "pickUpNote");

            currentCommand = new ParallelRaceGroup(
                new NotePresent(noteDetector, intake, swerveDrive, getIndex(cmd), false),
                new IntakeNote(swerveDrive, intake, noteDetector)
            );
        }

        else if (i+1 < args.length && !(args[i+1].substring(0, 2).equals("To") || args[i+1].substring(0, 2).equals("Fr"))) {
            Logger.recordOutput("AutoCommand/currentCommand", "replan");

            Translation2d notePose = Constants.FieldConstants.kNOTE_ARR[getIndex(args[i+1])].toTranslation2d();
            
            Translation2d offset;
            if (swerveDrive.getPose().getY() > notePose.getY()) {
                offset = new Translation2d(-0.5, 0.5);
            }
            else {
                offset = new Translation2d(-0.5, -0.5);
            }

            if (isRed) {
                offset = new Translation2d(-offset.getX(), offset.getY());
            }

            Translation2d finalT = notePose.plus(offset);

            currentCommand = DriveToPose.getCommand(
                new Pose2d(
                    finalT, 
                    new Rotation2d(
                        Math.atan2(finalT.getY() - notePose.getY(), finalT.getX() - notePose.getX())))
            );
        }

        currentCommand.schedule();

        i++;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private int getIndex(String cmd) {
        int index;
        // LOL
        switch (cmd) {
            case "far1":
                index = 0;
                break;

            case "far2":
                index = 1;
                break;

            case "far3":
                index = 2;
                break;

            case "far4":
                index = 3;
                break;
                
            case "far5":
                index = 4;
                break;
            
            case "amp":
                index = 5;
                break;

            case "mid":
                index = 6;
                break;
            
            case "source":
                index = 7;
                break;

            default:
                index = 0;
                break;
        }

        return index;
    }
}
