package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoRunner;
import frc.robot.commands.compositions.CancelIntakeNote;
import frc.robot.commands.compositions.FeedAndHoldNote;
import frc.robot.commands.compositions.IntakeNote;
import frc.robot.commands.compositions.ScoreAMP;
import frc.robot.commands.compositions.ShootNoteTele;
import frc.robot.commands.drivetrain.AbsoluteFieldDrive;
import frc.robot.commands.elevator.MoveElevatorToggle;
import frc.robot.commands.intake.RollIntakeEject;
import frc.robot.commands.intake.RollIntakeIn;
import frc.robot.commands.shooterComp.ShooterStop;
import frc.robot.commands.shooterComp.ShooterWindup;
import frc.robot.commands.shooterComp.ShooterWindupLob;
import frc.robot.lib.input.XboxController;
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
import frc.robot.subsystems.pivotComp.Pivot;
import frc.robot.subsystems.pivotComp.PivotIO;
import frc.robot.subsystems.pivotComp.PivotIONeo;
import frc.robot.subsystems.pivotComp.PivotIOSim;
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


  public RobotContainer() {
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    swerveDrive = new SwerveDrive();
    // flywheelSubsystem = new Flywheel();
    noteDetector = new NoteDetector(swerveDrive);
    indexer = new Indexer(swerveDrive, noteDetector);

    switch (Constants.currentMode) {
      case SIM:
        pivot = Pivot.setInstance(new Pivot(new PivotIOSim())); 
        intake = Intake.setInstance(new Intake(new IntakeIOSim(indexer))); //Assigns the instance object(pointer) to the variable so no new changes are needed.
        shooter = Shooter.setInstance(new Shooter(new ShooterIOSim(indexer)));
        elevator = Elevator.setInstance(new Elevator(new ElevatorIOSim()));

        break;
    
      case REAL:
        intake = Intake.setInstance(new Intake(new IntakeIONeo(indexer)));
        pivot = Pivot.setInstance(new Pivot(new PivotIONeo()));
        shooter = Shooter.setInstance(new Shooter(new ShooterIONeo(indexer)));
        elevator = Elevator.setInstance(new Elevator(new ElevatorIOFalcon()));

        break;

      default:
        pivot = Pivot.setInstance(new Pivot(new PivotIO(){}));
        intake = Intake.setInstance(new Intake(new IntakeIO(){}));
        shooter = Shooter.setInstance(new Shooter(new ShooterIO(){}));
        elevator = Elevator.setInstance(new Elevator(new ElevatorIO(){}));  

        break;
    }


    // intake = new Intake(indexer);
    indexer.setSubsystem(intake, pivot);


    autoRunner = new AutoRunner(swerveDrive);
    
    swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(swerveDrive, 
    () -> MathUtil.applyDeadband(-xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(-xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
    () -> MathUtil.applyDeadband(-xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND)));


    // OP Controlls
    SequentialCommandGroup intakeG, feedHold;
    intakeG = null;
    this.xboxOperator.getRightBumper().onTrue(new ShooterWindup());
    this.xboxOperator.getXButton().onTrue(new MoveElevatorToggle());
    this.xboxOperator.getYButton().onTrue(new ScoreAMP()); // changed
    this.xboxOperator.getAButton().onTrue(new ShootNoteTele()); // change back to shootNoteTele
    this.xboxOperator.getBButton().onTrue(feedHold = new FeedAndHoldNote());  
    this.xboxOperator.getLeftBumper().onTrue(new ShooterStop(feedHold));
    // this.xboxOperator.getRightMiddleButton().onTrue(new ParallelCommandGroup(new MoveElevatorAMP(), new IntakeNote()));
    this.xboxOperator.getLeftMiddleButton().onTrue(new ShooterWindupLob());
    this.xboxOperator.getRightMiddleButton().onTrue(new RollIntakeEject());

    // driver controlls
    this.xboxDriver.getXButton().onTrue(new InstantCommand(() -> swerveDrive.zeroGyro()));
    this.xboxDriver.getLeftBumper().whileTrue(new IntakeNote(swerveDrive, intake, noteDetector, pivot));
    this.xboxDriver.getRightMiddleButton().onTrue(new RollIntakeEject());
    this.xboxDriver.getRightBumper().onTrue(new CancelIntakeNote(intakeG, feedHold));
    this.xboxDriver.getLeftMiddleButton().onTrue(new InstantCommand(()-> pivot.zeroAngle()));
    this.xboxDriver.getYButton().onTrue(new InstantCommand(()-> Pivot.getInstance().TorusAngleReset()));

    // SYSID STUFF
    xboxTester.getAButton().whileTrue(swerveDrive.sysIDriveQuasistatic(Direction.kForward));
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
    return autoRunner.getAutonomousCommand();
  }

  public String getSelectedAuto() {
    return autoRunner.getSelectedAutoName();
  }

}

