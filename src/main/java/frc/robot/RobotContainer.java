package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoRunner;
import frc.robot.commands.compositions.IntakeNote;
import frc.robot.commands.drivetrain.AbsoluteFieldDrive;
import frc.robot.commands.intake.RollIntakeIn;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindup;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.drivetrain.vision.NoteDetector;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakeComp.Intake;
import frc.robot.subsystems.intakeComp.IntakeIO;
import frc.robot.subsystems.intakeComp.IntakeIONeo;
import frc.robot.subsystems.intakeComp.IntakeIOSim;
import frc.robot.subsystems.pivotComp.Pivot;
import frc.robot.subsystems.pivotComp.PivotIO;
import frc.robot.subsystems.pivotComp.PivotIONeo;
import frc.robot.subsystems.pivotComp.PivotIOSim;
import frc.robot.subsystems.shooter.pivot.flywheel.Flywheel;

public class RobotContainer {
  public static RobotContainer instance = null;

  private final XboxController xboxTester;
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;

  private final Flywheel flywheelSubsystem;

  private final SwerveDrive swerveDrive;

  private final AutoRunner autoRunner;

  private final NoteDetector noteDetector;

  private final Intake intake; // from comp
  private final Pivot pivot;

  private final Indexer indexer;


  public RobotContainer() {
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    swerveDrive = new SwerveDrive();
    flywheelSubsystem = new Flywheel();
    noteDetector = new NoteDetector(swerveDrive);
    indexer = new Indexer(swerveDrive, noteDetector);

    switch (Constants.currentMode) {
      case SIM:
        pivot = Pivot.setInstance(new Pivot(new PivotIOSim())); 
        intake = Intake.setInstance(new Intake(new IntakeIOSim(indexer))); //Assigns the instance object(pointer) to the variable so no new changes are needed.
        break;
    
      case REAL:
        intake = Intake.setInstance(new Intake(new IntakeIONeo(indexer)));
        pivot = Pivot.setInstance(new Pivot(new PivotIONeo()));
        break;

      default:
        pivot = Pivot.setInstance(new Pivot(new PivotIO(){}));
        intake = Intake.setInstance(new Intake(new IntakeIO(){}));
        break;
    }


    // intake = new Intake(indexer);
    indexer.setSubsystem(intake, pivot);


    autoRunner = new AutoRunner(swerveDrive);
    
    swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(swerveDrive, 
    () -> MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
    () -> MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND)));

    xboxDriver.getAButton().whileTrue(new IntakeNote(swerveDrive, intake, noteDetector, pivot));

    // xboxOperator.getAButton().whileFalse(new ShooterStop(flywheelSubsystem));

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

