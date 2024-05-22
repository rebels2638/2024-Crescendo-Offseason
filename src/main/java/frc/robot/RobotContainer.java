package frc.robot;

import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindup;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.shooter.pivot.flywheel.Flywheel;

public class RobotContainer {
  public static RobotContainer instance = null;

  private final XboxController xboxTester;
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;

  private Flywheel flywheelSubsystem;

  public RobotContainer() {
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    flywheelSubsystem = new Flywheel();

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

