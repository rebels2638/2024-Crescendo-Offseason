// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.commands.autoAligment.DriveToPose;
import frc.robot.commands.autoAligment.NotePresent;
import frc.robot.commands.compositions.IntakeNote;
import frc.robot.commands.compositions.ShootNoteAuto;
import frc.robot.commands.drivetrain.DriveToNote;
import frc.robot.commands.intake.InIntake;
import frc.robot.commands.intake.OutIntake;
import frc.robot.commands.intake.RollIntakeIn;
import frc.robot.commands.intake.RollIntakeInSlow;
import frc.robot.commands.intake.RollIntakeOut;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.pivot.PivotToTorus;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.drivetrain.vision.NoteDetector;
import frc.robot.subsystems.intakeComp.Intake;

import java.util.ArrayList;

import com.fasterxml.jackson.databind.SequenceWriter;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command Far1(SwerveDrive swerveDrive, Intake intake, NoteDetector noteDetector) {
    return new AutoCommand(new String[] {"ToFar1FromAMP", "far1", "far2", "far3", "ToAMPFromFar1"}, swerveDrive, intake, noteDetector);
  }

  public static Command configAuto(SwerveDrive swerveDrive) {
    Pose2d p = PathPlannerPath.fromPathFile("configtpath").getPreviewStartingHolonomicPose();
    return new SequentialCommandGroup(      
      new InstantCommand(() -> swerveDrive.resetPose(new Pose2d(p.getTranslation(), p.getRotation()/*new Rotation2d(p.getRotation().unaryMinus().getRadians()+Math.PI/2)))*/))),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("configtpath")),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("configt2path")));

  }

  public static Command fourPeiceMid(SwerveDrive swerveDrive, Intake intake, NoteDetector noteDetector) {
    Pose2d p = PathPlannerPath.fromPathFile("ToMidNoteFromMid").getPreviewStartingHolonomicPose();
    return new SequentialCommandGroup(      
      new InstantCommand(() -> swerveDrive.resetPose(new Pose2d(p.getTranslation(), p.getRotation()/*new Rotation2d(p.getRotation().unaryMinus().getRadians()+Math.PI/2)))*/))),
      new ParallelCommandGroup(
                    new RollIntakeIn(), // Begin rolling the intake in.
                    new PivotToTorus() // Pivot to the torus position.
                ),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("ToMidNoteFromMid")),
      new DriveToNote(swerveDrive, Intake.getInstance(), noteDetector),
      new ParallelCommandGroup(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("FromMidNoteToMid")),
        new SequentialCommandGroup(
          new StopIntake(), // Stop the intake mechanism.
                new PivotTurtle(), // Pivot to a turtle position.
                new RollIntakeOut(), // Roll the intake out.
                new WaitCommand(0.15), // Wait for 0.15 seconds.
                new OutIntake(), // Push the object out of the intake.
                new StopIntake(), // Stop the intake again.
                new RollIntakeInSlow(), // Slowly roll the intake in.
                new InIntake(), // Bring the object into the intake.
                new WaitCommand(0.1), // Wait for 0.1 seconds.
                new StopIntake()
        )
      ),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("ToSourceNoteFromMid")),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("FromSourceNoteToMid")),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("ToAMPNoteFromMidTurn")),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("FromAMPNoteToMidTurn")));
  }

  public static Command adaptableTest(SwerveDrive swerveDrive, Intake intake, NoteDetector noteDetector) {
    Pose2d p = PathPlannerPath.fromPathFile("ToAmpNoteFromAmpShort").getPreviewStartingHolonomicPose();
    // return new SequentialCommandGroup(null)

    Command midNote = new SequentialCommandGroup(
          DriveToPose.getCommand(new Pose2d(new Translation2d(2.62, 6.64), new Rotation2d(Math.toRadians(76.62)))),
          new ConditionalCommand(
            new ParallelRaceGroup(
              new IntakeNote(swerveDrive, intake, noteDetector),
              new NotePresent(noteDetector, intake, swerveDrive, 6, false)
            ),
            new InstantCommand(), 
            () -> !noteDetector.checked(6)
          )
    );
    

    Command midNoteInter = midNote.asProxy();

    Command ampNote = new ParallelRaceGroup(
              new IntakeNote(swerveDrive, intake, noteDetector),
              new NotePresent(noteDetector, intake, swerveDrive, 5, false)
    );

    
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveDrive.resetPose(new Pose2d(p.getTranslation(), new Rotation2d(p.getRotation().unaryMinus().getRadians()+Math.PI/2)))),
      // new InstantCommand(() -> System.out.println(new Rotation2d(p.getRotation().unaryMinus().getRadians()+Math.PI/2).getDegrees())),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("ToAmpNoteFromAmpShort")),
      new ConditionalCommand(
        ampNote,
        midNote,
        () -> !noteDetector.checked(5)
      ),
      new ConditionalCommand(
        midNoteInter,
        new InstantCommand(),
        () -> noteDetector.checked(5) && !noteDetector.checked(6)
      ),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("ToAmpFromAmpNoteShort"))
    );
  }
}