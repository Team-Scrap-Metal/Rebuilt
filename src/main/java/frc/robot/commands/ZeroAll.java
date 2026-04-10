// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Feeder.Feeder;
import frc.robot.Subsystems.Intake.Drum.Drum;
import frc.robot.Subsystems.Intake.Roller.Roller;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spindexer.Spindexer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroAll extends ParallelCommandGroup {
  /** Creates a new ZeroAll. */
  public ZeroAll(Drive drive, Feeder feeder, Drum drum, Roller roller, Shooter shooter, Spindexer spindexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("Disabled, zeroing all mechanisms"),
      DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0.0, () -> 0.0),
      new InstantCommand(
        () -> feeder.setFeederPercent(0), 
        feeder),
      new InstantCommand(
        () -> drum.setDrumPercent(0), 
        drum),
      new InstantCommand(
        () -> roller.setRollerPercent(0), 
        roller),
      new InstantCommand(
        () -> shooter.setShooterPercent(0), 
        shooter),
      new InstantCommand(
        () -> spindexer.setSpindexerPercent(0), 
        spindexer)
    );
  }
}
