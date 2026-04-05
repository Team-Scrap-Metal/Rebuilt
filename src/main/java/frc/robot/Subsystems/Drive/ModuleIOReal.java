// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Subsystems.Drive;

import frc.robot.util.SparkUtil;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.util.PhoenixUtil;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOReal implements ModuleIO {

  private final LoggedNetworkNumber DriveKpInput;
  private final LoggedNetworkNumber DriveKdInput;
  private final LoggedNetworkNumber DriveKsInput;
  private final LoggedNetworkNumber DriveKvInput;

  private final LoggedNetworkNumber TurnKpInput;
  private final LoggedNetworkNumber TurnKdInput;

  private double lastDriveKp;
  private double lastDriveKd;
  private double lastDriveKs;
  private double lastDriveKv;
  private double lastTurnKp;
  private double lastTurnKd;

  private final Rotation2d zeroRotation;

  // Hardware objects
  private final TalonFX driveTalon;
  private final SparkBase turnSpark;
  private final Rotation2d absoluteEncoderOffset;
  private final SimpleMotorFeedforward driveFeedForward;
  private final PIDController drivePIDController;
  private final PIDController steerPIDController;
  private Rotation2d absoluteTurnPosition;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Closed loop controllers
//   private final SparkClosedLoopController turnController;

  private CANcoder absoluteEncoder;

  // Queue inputs from odometry thread
//   private final Queue<Double> timestampQueue;

  private final StatusSignal<Angle> drivePosition;
  // private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOReal(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> Rotation2d.kZero;
        };

    driveTalon =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            }
        );
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    
    absoluteEncoder = new CANcoder(
        switch (module) {
            case 0 -> frontLeftAbsoluteCanId;
            case 1 -> frontRightAbsoluteCanId;
            case 2 -> backLeftAbsoluteCanId;
            case 3 -> backRightAbsoluteCanId;
            default -> 0;
        });
    
        absoluteEncoderOffset = switch (module) {
            case 0 -> frontLeftZeroRotation;
            case 1 ->  frontRightZeroRotation;
            case 2 ->  backLeftZeroRotation;
            case 3 ->  backRightZeroRotation;
            default -> new Rotation2d();
        };

    DriveKpInput = new LoggedNetworkNumber("/Tuning/Drive/Module_" + module + "_DKp", DriveConstants.driveKp[module]);
    DriveKdInput = new LoggedNetworkNumber("/Tuning/Drive/Module_" + module + "_DKd", DriveConstants.driveKd[module]);
    DriveKsInput = new LoggedNetworkNumber("/Tuning/Drive/Module_" + module + "_DKs", DriveConstants.driveKs[module]);
    DriveKvInput = new LoggedNetworkNumber("/Tuning/Drive/Module_" + module + "_DKv", DriveConstants.driveKv[module]);   

    TurnKpInput = new LoggedNetworkNumber("/Tuning/Drive/Module_" + module + "_TKp", DriveConstants.turnKp[module]);
    TurnKdInput = new LoggedNetworkNumber("/Tuning/Drive/Module_" + module + "_TKd", DriveConstants.turnKd[module]);

    lastDriveKp = DriveConstants.driveKp[module];
    lastDriveKd = DriveConstants.driveKd[module];
    lastDriveKs = DriveConstants.driveKs[module];
    lastDriveKv = DriveConstants.driveKv[module];
    lastTurnKp = DriveConstants.turnKp[module];
    lastTurnKd = DriveConstants.turnKd[module];

    drivePIDController = new PIDController(DriveConstants.driveKp[module], 0, DriveConstants.driveKd[module]);
    driveFeedForward = new SimpleMotorFeedforward(DriveConstants.driveKs[module], DriveConstants.driveKv[module]);
    steerPIDController = new PIDController(DriveConstants.turnKp[module], 0, DriveConstants.turnKd[module]);

    // turnEncoder = turnSpark.getAbsoluteEncoder();
    // driveController = driveTalon.getClosedLoopController();
    // turnController = turnSpark.getClosedLoopController();
        
    // Configure drive motor
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = driveMotorCurrentLimit;
    driveConfig.MotorOutput.Inverted = driveInverted[module]
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    // turnConfig
    //     .absoluteEncoder
    //     .inverted(turnEncoderInverted)
    //     .positionConversionFactor(turnEncoderPositionFactor)
    //     .velocityConversionFactor(turnEncoderVelocityFactor)
    //     .averageDepth(2);
    // turnConfig
    //     .closedLoop
    //     .feedbackSensor(FeedbackSensor.kNoSensor)
    //     .positionWrappingEnabled(true)
    //     .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
    //     .pid(0.0, 0.0, 0.0);
        
    drivePIDController.setPID(driveKp[module], 0.0, driveKd[module]);
    driveFeedForward.setKs(driveKs[module]);
    driveFeedForward.setKv(driveKv[module]);
    steerPIDController.setPID(turnKp[module], 0.0, turnKd[module]);
    steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
        // .p
    // turnConfig
    //     .signals
    //     .absoluteEncoderPositionAlwaysOn(true)
    //     .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
    //     .absoluteEncoderVelocityAlwaysOn(true)
    //     .absoluteEncoderVelocityPeriodMs(20)
    //     .appliedOutputPeriodMs(20)
    //     .busVoltagePeriodMs(20)
    //     .outputCurrentPeriodMs(20);
    SparkUtil.tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create odometry queues
    // timestampQueue = odometryThread.getInstance().makeTimestampQueue();
  
    // drivePositionQueue =
    //     odometryThread.getInstance().registerSignal(drivePos::getValueAsDouble);
    // turnPositionQueue =
    //     odometryThread.getInstance().registerSignal(turnSpark, ()-> absoluteEncoder.getAbsolutePosition().getValueAsDouble());

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        odometryFrequency, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent
        );
    driveTalon.optimizeBusUtilization();

  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signals
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

    // Update drive inputs
    SparkUtil.sparkStickyFault = false;
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    

    // Update turn inputs
    SparkUtil.sparkStickyFault = false;
    // SparkUtil.ifOk(
    //     turnSpark,
    //     turnEncoder::getPosition,
    //     (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    // SparkUtil.ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    SparkUtil.ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    SparkUtil.ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);
    inputs.turnPosition =
      new Rotation2d(MathUtil.angleModulus(
          Units.rotationsToRadians(
              absoluteEncoder.getAbsolutePosition().getValueAsDouble()))).minus(zeroRotation);
              
    absoluteTurnPosition = inputs.turnPosition;
    
    updatePidInputs();
    // Update odometry inputs
    // inputs.odometryTimestamps =
    //     timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    // inputs.odometryDrivePositionRad =
    //     driveTalon.getPosition().getValueAsDouble();
        // drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    // inputs.odometryTurnPosition =
    //     turnPositionQueue.stream()
    //         .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
    //         .toArray(Rotation2d[]::new);
    // timestampQueue.clear();
    // drivePositionQueue.clear();
    // turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    System.out.println(output);
    turnSpark.setVoltage(output);
}

@Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    
    driveTalon.setVoltage(driveFeedForward.calculate(velocityRadPerSec) + 
    drivePIDController.calculate(Units.rotationsToRadians(driveVelocity.getValueAsDouble()), velocityRadPerSec));
    
    // driveTalon.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec));
  }
  
  @Override
  public void setTurnPosition(Rotation2d rotation) {
      // System.out.println("Turn position: " + rotation);
      double setpoint =
      MathUtil.inputModulus(
          rotation.minus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    System.out.println(setpoint);
    System.out.println("P value " + steerPIDController.getP());
    turnSpark.setVoltage(
        steerPIDController
            .calculate(absoluteTurnPosition.getRadians(), setpoint)
            );
  }

  public void updatePidInputs() {
    double newDKp = DriveKpInput.get();
    double newDKd = DriveKdInput.get();
    double newDKs = DriveKsInput.get();
    double newDKv = DriveKvInput.get();
    double newTKp = TurnKpInput.get();
    double newTKd = TurnKdInput.get();

    if (newDKp != lastDriveKp || newDKd != lastDriveKd) {
      drivePIDController.setPID(newDKp, 0, newDKd);

      lastDriveKp = newDKp;
      lastDriveKd = newDKd;
    }

    if (newDKs != lastDriveKs || newDKv != lastDriveKv) {
      driveFeedForward.setKs(newDKs);
      driveFeedForward.setKv(newDKv);

      lastDriveKs = newDKs;
      lastDriveKv = newDKv;
    }

    if (newTKp != lastTurnKp || newTKd != lastTurnKd) {
      steerPIDController.setPID(newTKp, 0, newTKd);

      lastTurnKp = newTKp;
      lastTurnKd = newTKd;
    }
  }
}