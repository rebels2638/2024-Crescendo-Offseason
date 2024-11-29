package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoRunner;
import frc.robot.commands.autoAligment.DriveToPose;
import frc.robot.commands.autoAligment.NotePresent;
import frc.robot.commands.climber.MoveClimberDown;
import frc.robot.commands.climber.MoveClimberUp;
import frc.robot.commands.compositions.CancelIntakeNote;
import frc.robot.commands.compositions.FeedAndHoldNote;
import frc.robot.commands.compositions.IntakeNote;
import frc.robot.commands.compositions.IntakeNoteAuto;
import frc.robot.commands.compositions.IntakeNoteManual;
import frc.robot.commands.compositions.ScoreAMP;
import frc.robot.commands.compositions.ShootNote;
import frc.robot.commands.compositions.ShootNoteAuto;
import frc.robot.commands.compositions.ShootNoteTele;
import frc.robot.commands.drivetrain.AbsoluteFieldDrive;
import frc.robot.commands.elevator.MoveElevatorAMP;
import frc.robot.commands.elevator.MoveElevatorToggle;
import frc.robot.commands.elevator.MoveElevatorTurtle;
import frc.robot.commands.intake.InIntake;
import frc.robot.commands.intake.RollIntakeEject;
import frc.robot.commands.intake.RollIntakeIn;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.pivot.PivotToTorus;
import frc.robot.commands.shooterComp.ShooterAmp;
import frc.robot.commands.shooterComp.ShooterStop;
import frc.robot.commands.shooterComp.ShooterWindReverse;
import frc.robot.commands.shooterComp.ShooterWindup;
import frc.robot.commands.shooterComp.ShooterWindupAuto;
import frc.robot.commands.shooterComp.ShooterWindupLob;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOFalcon;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.drivetrain.vision.NoteDetector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOFalcon;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakeComp.Intake;
import frc.robot.subsystems.intakeComp.IntakeIO;
import frc.robot.subsystems.intakeComp.IntakeIONeo;
import frc.robot.subsystems.intakeComp.IntakeIOSim;
import frc.robot.subsystems.noteVisulaizer.NoteVisualizer;
import frc.robot.subsystems.noteVisulaizer.NoteVisualizer.Note;
import frc.robot.subsystems.pivotComp.Pivot;
import frc.robot.subsystems.pivotComp.PivotIO;
import frc.robot.subsystems.pivotComp.PivotIONeo;
import frc.robot.subsystems.pivotComp.PivotIOSim;
import frc.robot.subsystems.poseLimelight.PoseLimelight;
// import frc.robot.subsystems.shooter.pivot.flywheel.Flywheel;
import frc.robot.subsystems.shooterComp.Shooter;
import frc.robot.subsystems.shooterComp.ShooterIO;
import frc.robot.subsystems.shooterComp.ShooterIONeo;
import frc.robot.subsystems.shooterComp.ShooterIOSim;

public class RobotContainer {
  public static RobotContainer instance = null;

  private final XboxController xboxTester;
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;

  // private final Flywheel flywheelSubsystem;

  private final SwerveDrive swerveDrive;

  private final AutoRunner autoRunner;

  private final NoteDetector noteDetector;

  private final Intake intake; // from comp
  private final Pivot pivot;

  private final Indexer indexer;

  private final Elevator elevator;

  private final Shooter shooter;

  // private final PoseLimelight poseLimelight;

  public final NoteVisualizer noteVisualizer;
  // private final Climber climberSubsystem;

  public RobotContainer() {
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    // poseLimelight = new PoseLimelight();
    swerveDrive = new SwerveDrive(/*poseLimelight*/); //TODO: change null later :(
    // flywheelSubsystem = new Flywheel();
    noteDetector = new NoteDetector(swerveDrive);
    indexer = new Indexer(swerveDrive, noteDetector);

    switch (Constants.currentMode) {
      case SIM:
        pivot = Pivot.setInstance(new Pivot(new PivotIOSim())); 
        intake = Intake.setInstance(new Intake(new IntakeIOSim(indexer))); //Assigns the instance object(pointer) to the variable so no new changes are needed.
        shooter = Shooter.setInstance(new Shooter(new ShooterIOSim(indexer)));
        elevator = Elevator.setInstance(new Elevator(new ElevatorIOSim()));
        // climberSubsystem = Climber.setInstance(new Climber(new ClimberIOSim()));

        break;
    
      case REAL:
        intake = Intake.setInstance(new Intake(new IntakeIONeo(indexer)));
        pivot = Pivot.setInstance(new Pivot(new PivotIONeo()));
        shooter = Shooter.setInstance(new Shooter(new ShooterIONeo(indexer)));
        elevator = Elevator.setInstance(new Elevator(new ElevatorIOFalcon()));
        // climberSubsystem = Climber.setInstance(new Climber(new ClimberIO(){}));

        break;

      default:
        pivot = Pivot.setInstance(new Pivot(new PivotIO(){}));
        intake = Intake.setInstance(new Intake(new IntakeIO(){}));
        shooter = Shooter.setInstance(new Shooter(new ShooterIO(){}));
        elevator = Elevator.setInstance(new Elevator(new ElevatorIO(){}));  
        // climberSubsystem = Climber.setInstance(new Climber(new ClimberIOFalcon()));

        break;
    }


    // intake = new Intake(indexer);
    indexer.setSubsystem(intake, pivot);
    noteVisualizer = new NoteVisualizer();

    autoRunner = new AutoRunner(swerveDrive);

    NamedCommands.registerCommand("MoveElevatorAMP", new MoveElevatorAMP());
    NamedCommands.registerCommand("MoveElevatorTurtle", new MoveElevatorTurtle());
    NamedCommands.registerCommand("ShooterWindUp", new ShooterWindupAuto());
    NamedCommands.registerCommand("RollIntakeIn", new RollIntakeIn());
    NamedCommands.registerCommand("RollIntakeEject", new RollIntakeEject());
    NamedCommands.registerCommand("StopIntake", new StopIntake());
    NamedCommands.registerCommand("IntakeNote", new IntakeNoteAuto());
    NamedCommands.registerCommand("ShooterStop", new ShooterStop());
    NamedCommands.registerCommand("ShooterWindReverse", new ShooterWindReverse());
    NamedCommands.registerCommand("ShootNote", new ShootNote());
    NamedCommands.registerCommand("ShootNoteAuto", new ShootNoteAuto());
    NamedCommands.registerCommand("PivotToTorus", new PivotToTorus());
    NamedCommands.registerCommand("CancelIntakeNote", new CancelIntakeNote(null, null, swerveDrive));
    NamedCommands.registerCommand("RollIntakeIn", new RollIntakeIn());
    NamedCommands.registerCommand("LobNoteAuto", new SequentialCommandGroup(
                                                        new ParallelCommandGroup(new WaitCommand(0.4), new ShooterWindup(30)),
                                                        new RollIntakeIn(),
                                                        new StopIntake(),
                                                        new ShooterStop()));
    NamedCommands.registerCommand("VariableShoot", new InstantCommand(() -> Shooter.getInstance().setVelocityRadSec(0, true, 65, 17.5)));
    NamedCommands.registerCommand("InIntake", new InIntake());
    NamedCommands.registerCommand("AmpNotePresent", new NotePresent(noteDetector, intake, swerveDrive, 5, false));
    NamedCommands.registerCommand("MidNotePresent", new NotePresent(noteDetector, intake, swerveDrive, 6, false));
    NamedCommands.registerCommand("AmpNoteNotPresent", new NotePresent(noteDetector, intake, swerveDrive, 5, true));

    NamedCommands.registerCommand("DriveToMidNoteFromAmp", DriveToPose.getCommand(new Pose2d(new Translation2d(2.62, 6.64), new Rotation2d(Math.toRadians(-75)))));


    // OP Controlls
    SequentialCommandGroup intakeG, feedHold;
    intakeG = null;
    swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(swerveDrive, xboxDriver));
    
    this.xboxOperator.getRightBumper().onTrue(new ShooterWindup());
    // this.xboxOperator.getRightBumper().onTrue(new ShooterAmp());
    this.xboxOperator.getXButton().onTrue(new MoveElevatorToggle());
    this.xboxOperator.getYButton().onTrue(new ScoreAMP()); // changed

    this.xboxOperator.getAButton().onTrue(
      //co author as daniel :3
      new ParallelCommandGroup(
        new ShootNoteTele(),
        new InstantCommand(() -> noteVisualizer.addNote(
          new Note(
            Timer.getFPGATimestamp(), 
            new Translation3d(
              swerveDrive.getPose().getTranslation().getX(),
              swerveDrive.getPose().getTranslation().getY(),
              .7
            ), 
            swerveDrive.getPose().getRotation().getRadians(), 
            .5, 
            10,
            swerveDrive.getMeasuredFeildRelativeSpeeds().vxMetersPerSecond,
            swerveDrive.getMeasuredFeildRelativeSpeeds().vyMetersPerSecond)
          ))
      )); // change back to shootNoteTele

    this.xboxOperator.getBButton().onTrue(feedHold = new FeedAndHoldNote());  
    this.xboxOperator.getLeftBumper().onTrue(new ShooterStop(feedHold));
    // this.xboxOperator.getRightMiddleButton().onTrue(new ParallelCommandGroup(new MoveElevatorAMP(), new IntakeNote()));
    this.xboxOperator.getLeftMiddleButton().onTrue(new ShooterWindupLob());
    this.xboxOperator.getRightMiddleButton().onTrue(new RollIntakeEject());

    // driver controlls
    this.xboxDriver.getXButton().onTrue(new InstantCommand(() -> swerveDrive.zeroGyro()));
    this.xboxDriver.getLeftBumper().onTrue(intakeG = new IntakeNote(swerveDrive, intake, noteDetector));
    
    this.xboxDriver.getAButton().whileTrue(
      // swerveDrive.getPose().getTranslation().getDistance(Constants.FieldConstants.AMP_ALIGN_POSE_BLUE.getTranslation()) <= 
      // Constants.Auton.MAX_ALIGN_DIST_METERS ?
      DriveToPose.getCommand(Constants.FieldConstants.AMP_ALIGN_POSE_BLUE)/* : 
      new Command() {}*/);

    // this.xboxDriver.getBButton().onTrue(new MoveClimberUp());
    // this.xboxDriver.getAButton().onTrue(new MoveClimberDown());

    // this.xboxDriver.getLeftBumper().onTrue(intakeG = new IntakeNoteManual()); //TODO: CHECK W KUSH

    this.xboxDriver.getRightMiddleButton().onTrue(new RollIntakeEject());
    this.xboxDriver.getRightBumper().onTrue(new CancelIntakeNote(intakeG, feedHold, swerveDrive));
    // this.xboxDriver.getLeftMiddleButton().onTrue(new InstantCommand(()-> pivot.zeroAngle()));
    this.xboxDriver.getYButton().onTrue(new InstantCommand(()-> Pivot.getInstance().TorusAngleReset()));

    // SYSID STUFF
    // xboxTester.getAButton().whileTrue(swerveDrive.sysIDriveQuasistatic(Direction.kForward));
    xboxTester.getAButton().whileTrue(DriveToPose.getCommand(new Pose2d(new Translation2d(7.11, 6.48), new Rotation2d(Math.toRadians(161.27)))));

    xboxTester.getBButton().whileTrue(swerveDrive.sysIDriveQuasistatic(Direction.kReverse));
    xboxTester.getXButton().whileTrue(swerveDrive.sysIdDriveDynamic(Direction.kForward));
    xboxTester.getYButton().whileTrue(swerveDrive.sysIdDriveDynamic(Direction.kReverse));

  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    
    
    return instance;
  }
  
  public Command getAutonomousCommand() {
    return autoRunner.getAutonomousCommand(swerveDrive, intake, noteDetector);
  }

  public String getSelectedAuto() {
    return autoRunner.getSelectedAutoName();
  }

  public void offsetAngle() {
    swerveDrive.resetPose(new Pose2d(
      swerveDrive.getPose().getTranslation(), 
      swerveDrive.getPose().getRotation().plus(new Rotation2d(Math.PI / 2))));
  }
}

