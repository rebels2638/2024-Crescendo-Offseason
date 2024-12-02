package frc.robot.subsystems.drivetrain.swerve.module;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.lib.util.RebelUtil;
import frc.robot.subsystems.drivetrain.swerve.Phoenix6Odometry;

public class ModuleIOTalonFX implements ModuleIO {
    private TalonFX driveMotor;
    private TalonFX steerMotor;

    private CANcoder steerEncoder;

    private final StatusSignal<Double> drivePositionStatusSignal;
    private final StatusSignal<Double> driveVelocityStatusSignal;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveSupplyCurrent;
    private final StatusSignal<Double> driveTemperature;

    private final StatusSignal<Double> steerPositionStatusSignal;
    private final StatusSignal<Double> steerVelocityStatusSignal;
    private final StatusSignal<Double> steerAppliedVolts;
    private final StatusSignal<Double> steerSupplyCurrent;
    private final StatusSignal<Double> steerTemperature;
    private final StatusSignal<Double> steerEncoderPositionStatusSignal;
    private final StatusSignal<Double> steerEncoderAbsolutePosition;

    private final PIDController steerFB;
    private final PIDController driveFB;

    private final SimpleMotorFeedforward steerFF;
    private final SimpleMotorFeedforward driveFF;

    private double currSteerPositionRad;
    private double currDriveVelocity;
    private double prevTime;
    private double currTime;

    public ModuleIOTalonFX(int id) {
        steerFB = new PIDController(4.5, 1, 0.043, .315); // 6 0 .01 //3, 0.0, 0.02
        driveFB = new PIDController(0, 0, 0); // 2 0 0

        steerFF = new SimpleMotorFeedforward(0.23, 0.06); // .23
        driveFF = new SimpleMotorFeedforward(0.22, 2.2, 0.09); // 0.22 2.16 0

        currDriveVelocity = 0.0;
        currSteerPositionRad = 0.0;

        prevTime = 0.0;
        currTime = 0.0;
        
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        switch (id) {
            case 0:
                steerEncoder = new CANcoder(10, "drivetrain");
                encoderConfig.MagnetSensor.MagnetOffset = Constants.DrivetrainConstants.kFRONT_LEFT_ANGLE_OFFSET_DEG / 360;

                steerMotor = new TalonFX(5, "drivetrain");
                driveMotor = new TalonFX(4, "drivetrain");

                steerMotor.setInverted(true);
                driveMotor.setInverted(false);
                break;
            case 1:
                steerEncoder = new CANcoder(9, "drivetrain");
                encoderConfig.MagnetSensor.MagnetOffset = Constants.DrivetrainConstants.kFRONT_RIGHT_ANGLE_OFFSET_DEG / 360;

                steerMotor = new TalonFX(3, "drivetrain");
                driveMotor = new TalonFX(2, "drivetrain");

                steerMotor.setInverted(true);
                driveMotor.setInverted(false);
                break;
            case 2:
                steerEncoder = new CANcoder(11, "drivetrain");
                encoderConfig.MagnetSensor.MagnetOffset = Constants.DrivetrainConstants.kBACK_LEFT_ANGLE_OFFSET_DEG / 360;

                steerMotor = new TalonFX(7, "drivetrain");
                driveMotor = new TalonFX(6, "drivetrain");

                steerMotor.setInverted(true);
                driveMotor.setInverted(false);
                break;

            case 3:
                steerEncoder = new CANcoder(8, "drivetrain");
                encoderConfig.MagnetSensor.MagnetOffset = Constants.DrivetrainConstants.kBACK_RIGHT_ANGLE_OFFSET_DEG / 360;

                steerMotor = new TalonFX(0, "drivetrain");
                driveMotor = new TalonFX(1, "drivetrain");

                steerMotor.setInverted(true);
                driveMotor.setInverted(false);
                break;
        }

        steerEncoder.getConfigurator().apply(encoderConfig);
        
        steerMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);

        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();
        driveTemperature = driveMotor.getDeviceTemp();

        steerAppliedVolts = steerMotor.getMotorVoltage();
        steerSupplyCurrent = steerMotor.getSupplyCurrent();
        steerTemperature = steerMotor.getDeviceTemp();

        steerEncoderAbsolutePosition = steerEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                driveAppliedVolts,
                driveSupplyCurrent,
                steerAppliedVolts,
                steerSupplyCurrent,
                driveTemperature,
                steerTemperature,
                steerEncoderAbsolutePosition);

        drivePositionStatusSignal = driveMotor.getPosition().clone();
        driveVelocityStatusSignal = driveMotor.getVelocity().clone();
        steerPositionStatusSignal = steerMotor.getPosition().clone();
        steerVelocityStatusSignal = steerMotor.getVelocity().clone();
        steerEncoderPositionStatusSignal = steerEncoder.getPosition().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                drivePositionStatusSignal,
                driveVelocityStatusSignal,
                steerPositionStatusSignal,
                steerVelocityStatusSignal,
                steerEncoderPositionStatusSignal);

        Phoenix6Odometry.getInstance().registerSignal(driveMotor, drivePositionStatusSignal);
        Phoenix6Odometry.getInstance().registerSignal(driveMotor, driveVelocityStatusSignal);
        Phoenix6Odometry.getInstance().registerSignal(steerMotor, steerPositionStatusSignal);
        Phoenix6Odometry.getInstance().registerSignal(steerMotor, steerVelocityStatusSignal);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
        steerEncoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        double driveRotations = BaseStatusSignal.getLatencyCompensatedValue(drivePositionStatusSignal, driveVelocityStatusSignal);
        double steerRotations = BaseStatusSignal.getLatencyCompensatedValue(steerPositionStatusSignal, steerVelocityStatusSignal);

        BaseStatusSignal.refreshAll(
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTemperature,
                steerAppliedVolts,
                steerSupplyCurrent,
                steerTemperature,
                steerEncoderAbsolutePosition);

        inputs.timestamp = HALUtil.getFPGATime() / 1.0e6;
        currTime = inputs.timestamp;

        inputs.drivePositionRad = Units.rotationsToRadians(driveRotations / Constants.DrivetrainConstants.kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO);
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocityStatusSignal.getValueAsDouble() / Constants.DrivetrainConstants.kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO);

        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveSupplyCurrent.getValueAsDouble();

        // TODO: adjust based on the gearing
        inputs.steerAbsolutePosition = Rotation2d.fromRotations(steerEncoderAbsolutePosition.getValueAsDouble());
        inputs.steerAbsolutePositionRadians = inputs.steerAbsolutePosition.getRadians();
        inputs.steerPosition = Rotation2d.fromRotations(steerRotations);
        inputs.steerPositionRad = Units.rotationsToRadians(steerRotations);
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocityStatusSignal.getValueAsDouble());

        inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
        inputs.steerCurrentDrawAmps = steerSupplyCurrent.getValueAsDouble();

        inputs.driveTemperature = driveTemperature.getValueAsDouble();
        inputs.steerTemperature = steerTemperature.getValueAsDouble();

        inputs.desiredDriveVelocityMetersPerSec = currDriveVelocity;
        inputs.desiredSteerPositionRad = currSteerPositionRad;

    }

    //TODO: validate this later
    // reviewed: correct, however, 2910 uses steerMotor.setControl(new MotionMagicVoltage(0).withPosition(velocityRadSec)); for their stuff
    // TODO: exponential motionamagic profiles

    @Override
    public void setState(SwerveModuleState state) {
        if (DriverStation.isTest()) {return;}

        double desiredSpeed = state.speedMetersPerSecond;
        double driveVolts = driveFB.calculate(currDriveVelocity, desiredSpeed);
        driveVolts += driveFF.calculate(currDriveVelocity, state.speedMetersPerSecond, (currTime - prevTime)); // currTime is always slightly behind what it should be b/c inherent
        driveVolts = RebelUtil.constrain(driveVolts, -Constants.DrivetrainConstants.kMAX_DRIVE_VOLTAGE, Constants.DrivetrainConstants.kMAX_DRIVE_VOLTAGE);
        driveMotor.setVoltage(driveVolts);

        double desiredAngle = state.angle.getRadians();
        double steerVolts = steerFB.calculate(currSteerPositionRad, desiredAngle);
        
        double deltaTheta = desiredAngle - currSteerPositionRad;
        deltaTheta = deltaTheta < -Math.PI? Math.PI * 2 + deltaTheta: deltaTheta > Math.PI? Math.PI * 2 - deltaTheta: deltaTheta;

        Logger.recordOutput("SwerveDrive/deltaTheta", deltaTheta);
        double velocityRadSec = (deltaTheta) / (currTime - prevTime);
        Logger.recordOutput("SwerveDrive/velocityRadSec", velocityRadSec);
        steerVolts += steerFF.calculate(velocityRadSec, Math.signum(steerVolts)); // TODO: this is scuffed
        steerVolts = RebelUtil.constrain(steerVolts, -Constants.DrivetrainConstants.kMAX_ANGLE_VOLTAGE, Constants.DrivetrainConstants.kMAX_ANGLE_VOLTAGE);
        steerMotor.setVoltage(steerVolts);

        currDriveVelocity = desiredSpeed;
        currSteerPositionRad = desiredAngle;

        prevTime = Timer.getFPGATimestamp();
    }

    public double getOdometryTimestamp() {return this.currTime;}

    @Override
    public void setDriveVoltage(double baseUnitMagnitude) {}

}