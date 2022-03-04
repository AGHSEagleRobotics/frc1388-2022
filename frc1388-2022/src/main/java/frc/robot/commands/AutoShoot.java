// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.yaml.snakeyaml.scanner.Constant;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoMoveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem.FeederFunctions;

public class AutoShoot extends CommandBase {
  
  private final ShooterFeederSubsystem m_ShooterFeederSubsystem;
  private final Timer m_timer = new Timer();
  private boolean m_isFeederOn = false;

  // private final PIDController m_PidController = new PIDController(AutoMoveConstants.P_VALUE, 0, 0);

  /** Creates a new AutoShoot. */
  public AutoShoot(ShooterFeederSubsystem shooterFeederSubsystem) {

    System.out.println("starting constructer");

    m_ShooterFeederSubsystem = shooterFeederSubsystem;

    addRequirements(shooterFeederSubsystem);

  }
  /*
  GOALS:
  spin up shooter motor
  transition and feeder
  turn all motors off
  */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterFeederSubsystem.setShooterEnabled(true);
    m_ShooterFeederSubsystem.setTargetRPM(4500); // FIXME magic number shooter rpm
    m_timer.stop();
    m_timer.reset();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ShooterFeederSubsystem.shooterSpeedIsReady() && !m_isFeederOn) {
      m_isFeederOn = true;
      m_timer.start();
    }

    if (m_isFeederOn && m_timer.get() < 2) { // FIXME magic number feeder run time in seconds
      m_ShooterFeederSubsystem.setFeederFunction(FeederFunctions.FORWARD);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterFeederSubsystem.setShooterEnabled(false);
    m_ShooterFeederSubsystem.setFeederFunction(FeederFunctions.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(2);
  }
}
