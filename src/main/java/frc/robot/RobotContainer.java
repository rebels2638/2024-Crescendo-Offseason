package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.commands.drivetrain.AbsoluteFieldDrive;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindup;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.shooter.pivot.flywheel.Flywheel;

public class RobotContainer {
  public static RobotContainer instance = null;

  private final XboxController xboxTester;
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;

  private Flywheel flywheelSubsystem;

  private SwerveDrive swerveDrive;

  public RobotContainer() {
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    
    swerveDrive = new SwerveDrive();
    flywheelSubsystem = new Flywheel();

    swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(swerveDrive, 
    () -> MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
    () -> MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND)));

    xboxOperator.getAButton().whileTrue(new ShooterWindup(flywheelSubsystem));
    xboxOperator.getAButton().whileFalse(new ShooterStop(flywheelSubsystem));

  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }
  
}

