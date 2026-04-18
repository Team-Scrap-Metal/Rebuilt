// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  public ZeroAll(Spindexer spin, Feeder feed, Shooter shoot, Drum drum, Roller roll) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> spin.setSpindexerVoltage(0), spin),
      new InstantCommand(()-> feed.setFeederVoltage(0), feed),
      new InstantCommand(()-> shoot.setShooterVoltage(0), shoot),
      new InstantCommand(()-> drum.setDrumVoltage(0), drum),
      new InstantCommand(()-> roll.setRollerVoltage(0), roll)
    );
  }
}
