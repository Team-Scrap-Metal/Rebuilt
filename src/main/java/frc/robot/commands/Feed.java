// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Feeder.Feeder;
import frc.robot.Subsystems.Feeder.FeederConstants;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spindexer.Spindexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Feed extends Command {
  Spindexer m_spindexer;
  Feeder m_feeder;
  Shooter m_shooter;
  /** Creates a new Feed. */
  public Feed(Feeder feeder, Spindexer spindexer, Shooter shooter) {
    m_feeder = feeder;
    m_spindexer = spindexer;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooter.getVelocity() > 800) {
      m_feeder.setFeederPercent(FeederConstants.FEEDING_PERCENT);
      m_spindexer.setSpindexerPercent(FeederConstants.FEEDING_PERCENT);
    } else {
      m_feeder.setFeederPercent(0);
      m_spindexer.setSpindexerPercent(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
