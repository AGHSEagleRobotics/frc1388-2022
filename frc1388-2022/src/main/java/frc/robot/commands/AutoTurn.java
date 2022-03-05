// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoTurn extends CommandBase {

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final double m_rotation;
  private final double m_turnAngle;

  /** Creates a new AutoTurn. */
  public AutoTurn(DriveTrainSubsystem driveTrainSubsystem, double rotation, double turnAngle) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_rotation = rotation;
    m_turnAngle = turnAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrainSubsystem.getGyro();
  }

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
