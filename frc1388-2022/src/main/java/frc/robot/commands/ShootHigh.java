// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem.FeederFunctions;

public class ShootHigh extends CommandBase {
  private final ShooterFeederSubsystem m_shooterFeederSubsystem;
  private final TransitionSubsystem m_transitionSubsystem;
  private LED m_ledSybsystem;
  /** Creates a new ShooterCommands. */
  public ShootHigh(ShooterFeederSubsystem shooterSubsystem, TransitionSubsystem transitionSubsystem, LED led) {
    m_shooterFeederSubsystem = shooterSubsystem;
    m_transitionSubsystem = transitionSubsystem;
    m_ledSybsystem = led;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterFeederSubsystem.setShooterEnabled(true);
    m_shooterFeederSubsystem.setTargetRPM(ShooterConstants.SHOOTER_RPM_HIGHGOAL);
    m_ledSybsystem.ledShoot();
  }

  // For shooterSpeedIsReady, we need to start feeder when shooter speed is stable for around 1/5 of a second:

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // check with subystem to see if shooter ready
    if (m_shooterFeederSubsystem.shooterSpeedIsReady()) {
      //run feeder motor
      m_shooterFeederSubsystem.setFeederFunction(FeederFunctions.FORWARD);
      m_transitionSubsystem.setTransitionSpeed(TransitionConstants.TRANSITION_SPEED_FORWARD_FAST);
    } else {
      m_shooterFeederSubsystem.setFeederFunction(FeederFunctions.OFF);
      m_transitionSubsystem.setTransitionSpeed(TransitionConstants.TRANSITION_SPEED_FORWARD_SLOW);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Turn off feeder motor
    m_shooterFeederSubsystem.setFeederFunction(FeederFunctions.OFF);
    //Turn off shooter motor
    m_shooterFeederSubsystem.setShooterEnabled(false);
    m_transitionSubsystem.setTransitionSpeed(TransitionConstants.TRANSITION_SPEED_FORWARD_SLOW);
    m_ledSybsystem.ledNormal();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
