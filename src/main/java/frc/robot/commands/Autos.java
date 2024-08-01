// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.autoAligment.DriveToPose;
import frc.robot.commands.compositions.IntakeNote;
import frc.robot.commands.compositions.NotePresent;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.drivetrain.vision.NoteDetector;
import frc.robot.subsystems.intakeComp.Intake;

import com.pathplanner.lib.auto.AutoBuilder;
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

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command adaptableTest(SwerveDrive swerveDrive, Intake intake, NoteDetector noteDetector) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveDrive.resetPose(PathPlannerPath.fromPathFile("ToAmpNoteFromAmpShort").getPreviewStartingHolonomicPose())),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("ToAmpNoteFromAmpShort")),
      new ConditionalCommand(
        new ParallelRaceGroup(
              new IntakeNote(swerveDrive, intake, noteDetector),
              new NotePresent(noteDetector, intake, swerveDrive, 5, false)
        ),
        new SequentialCommandGroup(
          new DriveToPose(new Pose2d(new Translation2d(1.95, 6.44), new Rotation2d(Math.toRadians(123.35)))),
          new ConditionalCommand(
            new ParallelRaceGroup(
              new IntakeNote(swerveDrive, intake, noteDetector),
              new NotePresent(noteDetector, intake, swerveDrive, 6, false)
            ),
            new InstantCommand(() -> {}), 
            () -> !noteDetector.checked(6)
          )
        ),
        () -> /*!noteDetector.checked(5)*/ false
      ),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("ToAmpFromAmpNoteShort"))
    );
  }
}
