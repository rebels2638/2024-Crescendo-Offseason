// package frc.robot.subsystems.shooter.pivot.flywheel;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.lib.util.RebelUtil;

// public class Flywheel extends SubsystemBase {
//     private final FlywheeIOInputsAutoLogged inputs = new FlywheeIOInputsAutoLogged();
//     private FlywheeIO io;

//     private final SimpleMotorFeedforward realFF = new SimpleMotorFeedforward(0, 0, 0);
//     private final SimpleMotorFeedforward simFF = new SimpleMotorFeedforward(0, 0.00208, .012);
//     private SimpleMotorFeedforward feedforward;
    
//     private double desiredRPM = 0;
//     public Flywheel() {
//         switch(Constants.currentMode) {
//             case REAL:
//                 io = new FlywheeIOSparkMAX();
//                 feedforward = realFF;
//                 break;
//             case SIM:
//                 io = new FlywheeIOSim();
//                 feedforward = simFF;
//                 break;
//             case REPLAY_REAL:
//                 io = new FlywheeIO() {};
//                 feedforward = realFF;
//                 break;
//             default: // REPLAY_SIM
//                 io = new FlywheeIO() {};
//                 feedforward = simFF;
//                 break;
//         }
//     }

//     @Override
//     public void periodic() {
//         io.updateInputs(inputs);
//         Logger.processInputs("Flywheel", inputs);

//         double voltage = feedforward.calculate(desiredRPM, Math.signum(desiredRPM));
//         RebelUtil.constrain(voltage, -12, 12);

//         if (Math.max(inputs.tAmps, inputs.bAmps) > 40 || 
//             Math.max(inputs.tVolts, inputs.tVolts) > 12 || 
//             Math.max(inputs.tTemp, inputs.tTemp) >= 94) {
//             voltage = 0;
            
//             System.err.println("SHOOTER MOTOR NOT WITHIN OPERATION RANGE");
//             Logger.recordOutput("Flywheel/withinOperationRange", false);
//         }
//         else {
//             Logger.recordOutput("Flywheel/withinOperationRange", true);
//         }

//         io.setVoltage(voltage);
//         Logger.recordOutput("Flywheel/calculatedVoltage", voltage);

//     }

//     public void setRPM(double rpm) {
//         desiredRPM = rpm;
//         Logger.recordOutput("Flywheel/desiredRMP", desiredRPM);
//     }

//     public boolean reachedSetpoint() {
//         return Math.abs(inputs.RPM - desiredRPM) <= 5;
//     }
// }
