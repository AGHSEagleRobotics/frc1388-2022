// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoMoveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoLeave extends CommandBase {

  private final DriveTrainSubsystem m_driveTrainSubsystem;
  private final Timer m_timer = new Timer();

  private final PIDController m_PidController = new PIDController(AutoMoveConstants.P_VALUE, 0, 0);

  /** Creates a new AutoLeave. */
  public AutoLeave(DriveTrainSubsystem driveTrainSubsystem) {

  m_driveTrainSubsystem = driveTrainSubsystem;

  addRequirements(driveTrainSubsystem);

  m_PidController.setTolerance(0.5);
  }


  /*GOALS:
  Leave Tarmac length
  Track distance
  If distance, stop

  OR to start off
  go for a certain number of seconds
  */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasElapsed(2))
    m_driveTrainSubsystem.getCurrentCommand(); //I want it to do the default command up until 2, not like this
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
