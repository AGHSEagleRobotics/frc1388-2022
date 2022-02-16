// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoMove extends CommandBase {

  private DriveTrainSubsystem m_DriveTrainSubsystem;
  private final double m_setPoint;
  private final double m_speed;
  
  private final Timer m_timer = new Timer();

  /** Creates a new AutoMove. */
  public AutoMove(DriveTrainSubsystem driveTrainsSubsystem, double setPoint, double speed) {
    m_DriveTrainSubsystem = driveTrainsSubsystem;
    m_setPoint = setPoint;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveTrainsSubsystem);
  }

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
