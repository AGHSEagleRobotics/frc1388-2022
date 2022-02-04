// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem.FeederFunctions;

public class Shoot extends CommandBase {
  private final ShooterFeederSubsystem m_shooterSubsystem;
  /** Creates a new ShooterCommands. */
  public Shoot(ShooterFeederSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.shooterEnabled(true);

  }

  // For shooterSpeedIsReady, we need to start feeder when shooter speed is stable for around 1/5 of a second:

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // check with subystem to see if shooter ready
    if (m_shooterSubsystem.shooterSpeedIsReady()) {
      //run feeder motor
      m_shooterSubsystem.setFeederFunction(FeederFunctions.FORWARD);
    } else {
      m_shooterSubsystem.setFeederFunction(FeederFunctions.OFF);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Turn off feeder motor
    m_shooterSubsystem.setFeederFunction(FeederFunctions.OFF);
    //Turn off shooter motor
    m_shooterSubsystem.shooterEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
