// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;

public class AutoShoot2 extends CommandBase {

  private final DriveTrainSubsystem m_driveTrainSubsystem;
  private final ShooterFeederSubsystem m_shooterFeederSubsystem;

  /** Creates a new AutoShoot2. */
  public AutoShoot2(DriveTrainSubsystem driveTrainSubsystem, ShooterFeederSubsystem shooterFeederSubsystem) {
    m_shooterFeederSubsystem = shooterFeederSubsystem;
    m_driveTrainSubsystem = driveTrainSubsystem;

    addRequirements(shooterFeederSubsystem, driveTrainSubsystem);
    
    /*
    GOALS:
    Leave
    shoot
    pick up second (sensor?)


    */
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
