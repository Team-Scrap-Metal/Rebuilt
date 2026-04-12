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
import frc.robot.commands.DriveCommands;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  // private final Turret m_turret;

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
        // m_turret = new Turret(new TurretIOSpark());
        m_spindexer = new Spindexer(new SpindexerIOSpark());

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOReal(0),
                new ModuleIOReal(1),
                new ModuleIOReal(2),
                new ModuleIOReal(3));
        break;

      case SIM:
        m_feeder = new Feeder(new FeederIOSim());
        // m_spindexer = new Spindexer(new SpindexerIOSim());
        m_shooter = new Shooter(new ShooterIOSim());
        m_drum = new Drum(new DrumIOSim());
        m_roller = new Roller(new RollerIOSim());
        // m_turret = new Turret(new TurretIOSim());
        m_spindexer = new Spindexer(new SpindexerIOSim());

        drive =
          new Drive(
              new GyroIO() {},
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
        break;

      default:
        m_feeder = new Feeder(new FeederIOSim());
        // m_spindexer = new Spindexer(new SpindexerIOSim());
        m_shooter = new Shooter(new ShooterIOSim());
        m_drum = new Drum(new DrumIOSim());
        m_roller = new Roller(new RollerIOSim());
        // m_turret = new Turret(new TurretIOSim());
        m_spindexer = new Spindexer(new SpindexerIOSim());

        drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
        break;
    }

    m_pathplanner = new PathPlanner(drive, drive.getPoseEstimator());

        // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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

    m_driverController
      .leftTrigger()
      .onTrue(
          new  InstantCommand(
            () ->
            // m_shooter.shootFromDistance(m_shooter.getHubDistance())
            m_shooter.setShooterRPM(m_shooter.getTunedRPM())
          )
      )
      .onFalse(new ParallelCommandGroup(
        new InstantCommand(
          () ->
            m_shooter.setShooterPercent(0),
            m_shooter)));
    m_driverController
      .rightTrigger()
      .onTrue(
          new Feed(m_feeder, m_spindexer)
      )
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


        // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    // Lock to 0° when A button is held
    m_driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    // m_driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    m_driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> drive.zeroHeading()));
        //                 drive.setPose(
        //                     new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
        //             drive)
        //         .ignoringDisable(true));

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
          ),
          new InstantCommand(
            () -> m_shooter.runReverse(),
            m_shooter
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
          ),
          new InstantCommand(
            () -> m_shooter.setShooterPercent(0),
            m_shooter)));
    
    // Aux reverse bindings
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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return m_shooter.runFullSysId();
    return autoChooser.get();
  }
}
