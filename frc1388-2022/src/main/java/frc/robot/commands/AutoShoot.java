// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.AutoMoveConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem.FeederFunctions;

public class AutoShoot extends CommandBase {
  
  private static final Logger log = LogManager.getLogger(AutoShoot.class);

  private final ShooterFeederSubsystem m_shooterFeederSubsystem;
  private final TransitionSubsystem m_transitionSubsystem;
  // private final ShooterFeederSubsystem m_shootFeederSubsystem;
  private final double m_RPM;

  // private final PIDController m_PidController = new PIDController(AutoMoveConstants.P_VALUE, 0, 0);

  /** Creates a new AutoShoot. */
  public AutoShoot(ShooterFeederSubsystem shooterFeederSubsystem, TransitionSubsystem transitionSubsystem, double RPM) {

    m_shooterFeederSubsystem = shooterFeederSubsystem;
    m_transitionSubsystem = transitionSubsystem;
    m_RPM = RPM;

    addRequirements(shooterFeederSubsystem, transitionSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.info("m_RPM={}",m_RPM);
    m_shooterFeederSubsystem.setShooterEnabled(true);
    m_shooterFeederSubsystem.setTargetRPM(m_RPM);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    log.debug("TargetRPM: {} \tRealRPM: {}", m_RPM, m_shooterFeederSubsystem.getRealRPM());
    if (m_shooterFeederSubsystem.shooterSpeedIsReady()) {
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
    m_shooterFeederSubsystem.setShooterEnabled(false);
    m_shooterFeederSubsystem.setFeederFunction(FeederFunctions.OFF);
    m_transitionSubsystem.setTransitionSpeed(TransitionConstants.TRANSITION_SPEED_FORWARD_SLOW);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // boolean finished = (Timer.getMatchTime() > AutoConstants.AUTO_TIME);
    return false;
  }
}
