// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroIO;
import frc.robot.Subsystems.Drive.GyroIOPigeon2;
import frc.robot.Subsystems.Drive.ModuleIO;
import frc.robot.Subsystems.Drive.ModuleIOReal;
import frc.robot.Subsystems.Drive.ModuleIOSim;
import frc.robot.Subsystems.Feeder.*;
import frc.robot.Subsystems.Intake.Drum.*;
import frc.robot.Subsystems.Intake.Roller.*;
import frc.robot.Subsystems.Shooter.*;
import frc.robot.Subsystems.Spindexer.*;
import frc.robot.Subsystems.Turret.*;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionIO;
import frc.robot.Subsystems.Vision.VisionIOLimelight;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroIO;
import frc.robot.Subsystems.Drive.GyroIOPigeon2;
import frc.robot.Subsystems.Drive.ModuleIO;
import frc.robot.Subsystems.Drive.ModuleIOReal;
import frc.robot.Subsystems.Drive.ModuleIOSim;
import frc.robot.Subsystems.Feeder.*;
import frc.robot.Subsystems.Shooter.*;
import frc.robot.Subsystems.Spindexer.*;
import frc.robot.Subsystems.Turret.*;
import frc.robot.Subsystems.Intake.Pivot.*;
import frc.robot.commands.DriveCommands;

import java.lang.annotation.Target;
import java.time.Instant;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Feed;
import frc.robot.util.PathPlanner;
import frc.robot.util.PoseEstimator;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Feeder m_feeder;
  // private final Spindexer m_spindexer;
  private final Shooter m_shooter;
  private final Drum m_drum;
  private final Roller m_roller;
  private final Spindexer m_spindexer;
  private final Turret m_turret;
  private final Vision m_vision;
  private final Pivot m_pivot;


  private final PathPlanner m_pathplanner;
//   private final PoseEstimator m_poseEstimator;

  private final Drive drive;
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_auxController =
      new CommandXboxController(OperatorConstants.AUX_CONTROLLER_PORT);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private enum TargetState {
    HUB_SCORING,
    PASSING
  }

  private TargetState m_currentTargetingState;

  // private final LoggedNetworkNumber speedPercentInput = new LoggedNetworkNumber("/Tuning/FeederPercent", FeederConstants.FEEDING_PERCENT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        m_feeder = new Feeder(new FeederIOSpark());
        // m_spindexer = new Spindexer(new SpindexerIOSpark());
        m_shooter = new Shooter(new ShooterIOSpark());
        m_drum = new Drum(new DrumIOSpark());
        m_roller = new Roller(new RollerIOSpark());
        m_turret = new Turret(new TurretIOSpark());
        m_pivot = new Pivot(new PivotIOSpark());
        m_spindexer = new Spindexer(new SpindexerIOSpark());

        drive =
        new Drive(
          new GyroIOPigeon2(),
          new ModuleIOReal(0),
          new ModuleIOReal(1),
          new ModuleIOReal(2),
          new ModuleIOReal(3));
          
        m_vision =
            new Vision(
                drive.getPoseEstimator()::addVisionMeasurement,
                new VisionIOLimelight(drive::getRawGyroRotation));
        break;

      case SIM:
        m_feeder = new Feeder(new FeederIOSim());
        // m_spindexer = new Spindexer(new SpindexerIOSim());
        m_shooter = new Shooter(new ShooterIOSim());
        m_drum = new Drum(new DrumIOSim());
        m_roller = new Roller(new RollerIOSim());
        m_turret = new Turret(new TurretIOSim());
        m_pivot = new Pivot(new PivotIOSim());
        m_spindexer = new Spindexer(new SpindexerIOSim());

        drive =
          new Drive(
              new GyroIO() {},
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());

        m_vision = null;
        // m_vision =
        //   new Vision(
        //       drive.getPoseEstimator()::addVisionMeasurement,
        //       new VisionIOSim(drive::getRotation));

        break;

      default:
        m_feeder = new Feeder(new FeederIOSim());
        m_shooter = new Shooter(new ShooterIOSim());
        m_drum = new Drum(new DrumIOSim());
        m_roller = new Roller(new RollerIOSim());
        m_turret = new Turret(new TurretIOSim());
        m_spindexer = new Spindexer(new SpindexerIOSim());
        m_pivot = new Pivot(new PivotIOSim());

        drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});

        m_vision = null;
        break;
    }

    m_currentTargetingState = TargetState.HUB_SCORING;

    m_pathplanner = new PathPlanner(drive, drive.getPoseEstimator());

    NamedCommands.registerCommand("Intake", 
      new ParallelCommandGroup(
        new InstantCommand( () -> m_drum.drumIntake(), m_drum),
        new InstantCommand( () -> m_roller.runRoller(), m_roller)
      )
    );

    NamedCommands.registerCommand("ZeroIntake", 
      new ParallelCommandGroup(
        new InstantCommand( () -> m_drum.setDrumPercent(0), m_drum),
        new InstantCommand( () -> m_roller.setRollerPercent(0), m_roller)
      )
    );

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addDefaultOption(
      "8 Fuel Center Auto", 
      new SequentialCommandGroup(
        new InstantCommand ( () -> m_shooter.shootFromHub(), m_shooter),
        new WaitCommand(1),
        new Feed(m_feeder, m_spindexer),
        new WaitCommand(10),
        new ParallelCommandGroup(
          new InstantCommand( () -> m_feeder.setFeederPercent(0), m_feeder),
          new InstantCommand( () -> m_spindexer.setSpindexerPercent(0), m_spindexer)
        )
      ));

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    // Intake
    m_driverController
      .leftBumper()
      .onTrue(
        new ParallelCommandGroup(
          new InstantCommand(
            () ->
              m_drum.drumIntake(),
              m_drum),
          new InstantCommand(
            () ->
              m_roller.runRoller(),
              m_roller)
        ))
      .onFalse(
        new ParallelCommandGroup(
          new InstantCommand(
            () ->
              m_drum.setDrumPercent(0),
              m_drum),
          new InstantCommand(
            () ->
              m_roller.setRollerPercent(0),
              m_roller)
          ));

    // Ready shoot - spin up shooter and aim turret at hub
    m_driverController
      .leftTrigger()
      .onTrue(
          new InstantCommand(
            () -> m_shooter.shootAtHub(drive),
            m_shooter
            // m_shooter.setShooterRPM(m_shooter.getTunedRPM())
          )
        )
      .whileTrue(
        Commands.run(
          () -> m_turret.targetHub(drive),
          m_turret)
          )
      .onFalse(new ParallelCommandGroup(
        new InstantCommand(
          () ->
            m_shooter.setShooterPercent(0),
            m_shooter),
        new InstantCommand(
          () -> m_turret.setTurretPercent(0),
          m_turret
        )));

    // Intake launch
    m_driverController
      .rightBumper()
      .onTrue(
        new ParallelCommandGroup(
          new InstantCommand(
            () ->
              m_drum.drumLaunch(),
              m_drum),
          new InstantCommand(
            () ->
              m_roller.runRoller(),
              m_roller)
        ))
      .onFalse(
        new ParallelCommandGroup(
          new InstantCommand(
            () ->
              m_drum.setDrumPercent(0),
              m_drum),
          new InstantCommand(
            () ->
              m_roller.setRollerPercent(0),
              m_roller)
          ));

    // Shoot - feed spun up shooter
    m_driverController
      .rightTrigger()
      .onTrue(
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> m_feeder.runReverse(),
              m_feeder
            ),
            new InstantCommand(
              () -> m_spindexer.runReverse(),
              m_spindexer
            )
          ),
          new WaitCommand(0.5),
          new Feed(m_feeder, m_spindexer)
      ))
      .onFalse(new ParallelCommandGroup(
        new InstantCommand(
          () ->
            m_feeder.setFeederPercent(0),
            m_feeder),
        new InstantCommand(
          () ->
            m_spindexer.setSpindexerPercent(0),
            m_spindexer)
        )
      );
  
    // toggle passive intake down
    m_driverController
      .a()
      .onTrue(
        new InstantCommand(
          () -> m_pivot.togglePassiveDown(),
          m_pivot
      ));

    // TODO: Add binding to toggle passing/scoring 

    // Reverse feeding when x is pressed
    m_driverController
      .x()
      .onTrue(
        new ParallelCommandGroup(
          new InstantCommand(
            () -> m_feeder.runReverse(),
            m_feeder
          ),
          new InstantCommand(
            () -> m_spindexer.runReverse(),
            m_spindexer
          )
        )
      ).onFalse(
        new ParallelCommandGroup(
          new InstantCommand(
            () -> m_feeder.setFeederPercent(0),
            m_feeder
          ),
          new InstantCommand(
            () -> m_spindexer.setSpindexerPercent(0),
            m_spindexer
          )));

    m_driverController
      .y()
      .onTrue(
        new InstantCommand(() -> m_currentTargetingState =  m_currentTargetingState == TargetState.HUB_SCORING ? TargetState.PASSING : TargetState.HUB_SCORING)
      );


    // Intake pivoting up (stowing)/down (extending)
    m_driverController
      .povUp()
      .onTrue(
        new InstantCommand(()->
          m_pivot.runPivot(true),
          m_pivot
        ))
      .onFalse(
        new InstantCommand(()->
          m_pivot.setPivotPercent(0),
          m_pivot
        )
      );
    m_driverController
      .povDown()
      .onTrue(
        new InstantCommand(()->
          m_pivot.runPivot(false),
          m_pivot
        ))
      .onFalse(
        new InstantCommand(()->
          m_pivot.setPivotPercent(0),
          m_pivot
      ));

    // Reset heading
    m_driverController
        .povLeft()
        .onTrue(
            Commands.runOnce(
                    () -> drive.zeroHeading()));
        //                 drive.setPose(
        //                     new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
        //             drive)
        //         .ignoringDisable(true));

    m_turret.setDefaultCommand(
      m_turret.setTurretPositionWithController(
          m_turret,
          () -> m_auxController.getLeftX(),
          () -> -m_auxController.getLeftY(),
          drive
      )
    );

    // Toggle manual/auto turret control
    m_auxController
      .leftBumper()
      .onTrue(
        new InstantCommand(
          () -> m_turret.toggleManualControl(),
          m_turret
        ));

    // Run static RPM to score when robot left is near-flush with hub
    m_auxController
      .leftTrigger()
      .onTrue(
        new InstantCommand(
          () -> m_shooter.shootFromHub(),
          m_shooter
        ))
      .onFalse(
        new InstantCommand(
          () -> m_shooter.setShooterPercent(0),
          m_shooter
        )
      );

    /** Zero Turret Encoder */
    m_auxController
        .b()
        .onTrue(
          Commands.runOnce (
             () -> m_turret.zeroEncoder()
          )
        );

    // Reverse shooter
    m_auxController
      .povUp()
      .onTrue(
        new InstantCommand(
          () -> m_shooter.runReverse(),
          m_shooter))
      .onFalse(
        new InstantCommand(
          () -> m_shooter.setShooterPercent(0),
          m_shooter
        ));

    // reverse spindexer
    m_auxController
      .povDown()
      .onTrue(
        new InstantCommand(
          () -> m_spindexer.runReverse(),
          m_spindexer))
      .onFalse(
        new InstantCommand(
          () -> m_spindexer.setSpindexerPercent(0),
          m_spindexer
        ));

    // Reverse drum
    m_auxController
      .povLeft()
      .onTrue(
        new ParallelCommandGroup(
          new InstantCommand(
            () -> m_drum.runReverse(),
            m_drum
          ),
          new InstantCommand(
            () -> m_roller.runReverse(),
            m_roller
          )))
      .onFalse(
        new ParallelCommandGroup(
          new InstantCommand(
            () -> m_drum.setDrumPercent(0),
            m_drum),
          new InstantCommand(
            () -> m_roller.setRollerPercent(0),
            m_roller)));

    // Reverse feeder
    m_auxController
      .povRight()
      .onTrue(
        new InstantCommand(
          () -> m_feeder.runReverse(),
          m_feeder))
      .onFalse(
        new InstantCommand(
          () -> m_feeder.setFeederPercent(0),
          m_feeder
        ));
    
    // TODO: TEMP CODE REMOVE BEFORE UTAH
    m_auxController
      .rightTrigger()
      .onTrue(
        new InstantCommand(
          () -> m_shooter.shootAtTuned(),
          m_shooter
        )
      )
      .onFalse(
        new InstantCommand(
          () -> m_shooter.setShooterPercent(0),
          m_shooter
        )
      );
  }        

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return m_shooter.runFullSysId();
    // drive.getPoseEstimator().updateStartingPose();

    return new SequentialCommandGroup(
        new InstantCommand ( () -> m_shooter.shootFromHub(), m_shooter),
        new WaitCommand(2),
        new Feed(m_feeder, m_spindexer)
        // new WaitCommand(10),
        // new ParallelCommandGroup(
        //   new InstantCommand( () -> m_feeder.setFeederPercent(0), m_feeder),
        //   new InstantCommand( () -> m_spindexer.setSpindexerPercent(0), m_spindexer)
        // )
      );
  }

  public void disabledInit() {
    m_turret.setBrake(false);
  }
}