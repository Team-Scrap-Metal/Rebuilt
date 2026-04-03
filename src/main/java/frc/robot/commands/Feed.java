// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Feeder.Feeder;
import frc.robot.Subsystems.Feeder.FeederConstants;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.Spindexer.Spindexer;
import frc.robot.Subsystems.Spindexer.SpindexerConstants;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Feed extends ParallelCommandGroup {
  /** Creates a new Shoot. */
  public Feed(Feeder feeder, Spindexer spindexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(
          () ->
            feeder.setFeederPercent(feeder.getTunedPercent()),
            feeder),
        new InstantCommand(
          () ->
            spindexer.setSpindexerPercent(spindexer.getTunedPercent()),
            spindexer)
    );
  }
}
