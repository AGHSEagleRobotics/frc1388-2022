// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoMoveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;

public class AutoShoot extends CommandBase {
  
  private final DriveTrainSubsystem m_driveTrainSubsystem;
  private final ShooterFeederSubsystem m_ShooterFeederSubsystem;
  private final Timer m_timer = new Timer();

  private final PIDController m_PidController = new PIDController(AutoMoveConstants.P_VALUE, 0, 0);

  /** Creates a new AutoShoot. */
  public AutoShoot(DriveTrainSubsystem driveTrainSubsystem, ShooterFeederSubsystem shooterFeederSubsystem) {

    m_ShooterFeederSubsystem = shooterFeederSubsystem;
    m_driveTrainSubsystem = driveTrainSubsystem;

    addRequirements(driveTrainSubsystem, shooterFeederSubsystem);

    m_PidController.setTolerance(0.5);
  }
  /*
  GOALS:
  Drive to a certain location
  Shoot once at location
  */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
